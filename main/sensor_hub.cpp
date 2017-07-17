#include <cstdio>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>
#include <map>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"

#include "freertos_future.hpp"
#include "bluetooth_util.hpp"

static const BluetoothUuid target_uuid("F0000000-0451-4000-B000-000000000000");



using namespace std;
static const char* TAG = "SENSORHUB";
static const char TARGET_DEVICE_NAME[] = "CC2650 SensorTag";

static const uint16_t APP_ID = 1;
static esp_gatt_if_t app_gattc_if = ESP_GATT_IF_NONE;
static bool isTargetDetected = false;
static esp_bd_addr_t target_bdaddr = {0};

static bool get_localname_from_advdata(const uint8_t* advData, uint8_t advDataLength, char* buffer)
{
    for(const uint8_t* adStructure = advData; adStructure - advData < advDataLength; adStructure += adStructure[0]) {
        if( adStructure[0] == 0 ) break;
        
        uint8_t adType = adStructure[1];
        if( adType != 0x08 && adType != 0x09 ) continue;

        memcpy(buffer, adStructure+2, adStructure[0]-1);
        buffer[adStructure[0]-1] = 0;
        return true;
    }

    buffer[0] = 0;
    return false;
}

static unique_ptr<GattClient> gatt_client;
static bool isFirstGapEvent = true;
static shared_ptr<GattClientService> temperature_service;
static const BluetoothUuid TemperatureServiceUuid("F000AA00-0451-4000-B000-000000000000");
static const BluetoothUuid TemperatureDataCharacteristicUuid("F000AA01-0451-4000-B000-000000000000");
static const BluetoothUuid TemperatureConfigurationCharacteristicUuid("F000AA02-0451-4000-B000-000000000000");
static const BluetoothUuid NotificationDescriptorUuid(static_cast<uint16_t>(0x2902u), true);

static void handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch(event)
    {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        uint8_t* bda = param->scan_rst.bda;
        char localname[ESP_BLE_ADV_DATA_LEN_MAX];
        
        get_localname_from_advdata(param->scan_rst.ble_adv+param->scan_rst.adv_data_len, param->scan_rst.scan_rsp_len, localname);
        ESP_LOGI(TAG, "SCAN_RESULT: ADDR=%02x:%02x:%02x:%02x:%02x:%02x, NAME:%s, RSSI:%d", bda[0], bda[1],bda[2],bda[3],bda[4],bda[5], localname, param->scan_rst.rssi);
        if( strcmp(localname, TARGET_DEVICE_NAME) == 0 ) {
            ESP_LOGI(TAG, "Target device was detected, ADDR=%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1],bda[2],bda[3],bda[4],bda[5]);
            ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
            memcpy(target_bdaddr, bda, sizeof(esp_bd_addr_t));
			gatt_client->set_discovery_completed_handler([](GattClient& client) {
				ESP_LOGI(TAG, "Discovery completed");
				temperature_service = client.get_service(TemperatureServiceUuid);
				if (!temperature_service) {
					ESP_LOGE(TAG, "Failed to get temperature service.");
				}
			});
			gatt_client->open(BdAddr(target_bdaddr));
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
        ESP_LOGI(TAG, "BLE Scan started. status=%x", param->scan_start_cmpl.status);
        memset(target_bdaddr, 0, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        ESP_LOGI(TAG, "Scanning stopped.");
    }
    default: break;
    }
}

extern "C" void app_main();

void app_main()
{
    static esp_ble_scan_params_t scan_params;

    printf("Hello world!\n");
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    ESP_LOGI(TAG, "Initializing ESP Bluedroid...");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK);
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

	// Initialize GattClient
	GattClient::initialize();
	
    ESP_LOGI(TAG, "Initialzing ESP BLE...");
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(handle_gap_event));
    scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
    scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    scan_params.scan_interval = 1600;   // 1[s]
    scan_params.scan_window = 320;      // 200[ms]
    
	gatt_client.reset(new GattClient(1));

	printf("target_uuid: %s", target_uuid.str().c_str());

    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(60));   // Scan 60[s]

	while (!temperature_service) vTaskDelay(portTICK_PERIOD_MS);

	ESP_LOGI(TAG, "Enumerating temperature characteristics");
	auto result = temperature_service->enumerate_characteristics().get();
	ESP_LOGI(TAG, "Enumeration complete! result=%x", result);
	{
		ESP_LOGI(TAG, "Writing Configuration characteristic");
		auto characteristic = temperature_service->get_characteristic(TemperatureConfigurationCharacteristicUuid);
		uint8_t value = 0x01;
		auto result = characteristic->write_value_async(&value, 1).get();
		ESP_LOGI(TAG, "Writing value complete. status=%x", result);
	}
	{
		auto characteristic = temperature_service->get_characteristic(TemperatureDataCharacteristicUuid);
		ESP_LOGI(TAG, "Enumerating descriptors...");
		auto result = characteristic->enumerate_descriptors().get();
		ESP_LOGI(TAG, "Enumerating descriptors completed. status=%x", result);
		characteristic->set_notification_handler([](const uint8_t* data, size_t length) {
			ESP_LOGI(TAG, "Notification. length=0x%x", length);
			if (length == 4) {
				uint16_t raw_obj = data[0] | (data[1] << 8);
				uint16_t raw_amb = data[2] | (data[3] << 8);
				float obj = (raw_obj >> 2) * 0.03125f;
				float amb = (raw_amb >> 2) * 0.03125f;
				ESP_LOGI(TAG, "AMB:%f, OBJ:%f", amb, obj);
			}
		});
		result = characteristic->enable_notification_async(true).get();
		ESP_LOGI(TAG, "Notification enabled. status=%x", result);

		auto descriptor = characteristic->get_descriptor(NotificationDescriptorUuid);
		uint8_t value[2] = { 0x01, 0x00 };
		result = descriptor->write_value_async(value, sizeof(value)).get();
		ESP_LOGI(TAG, "Descriptor configured. status=%x", result);


		while (true) vTaskDelay(portTICK_PERIOD_MS * 1000);
	}
}
