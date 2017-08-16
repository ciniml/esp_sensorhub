#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>
#include <map>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#include <esp_wifi.h>

#include "freertos_future.hpp"
#include "bluetooth_util.hpp"
#include "tls_client.hpp"
#include "http_client.hpp"

static const BluetoothUuid target_uuid("F0000000-0451-4000-B000-000000000000");

using namespace std;
static const char* TAG = "SENSORHUB";
static const char TARGET_DEVICE_NAME[] = "CC2650 SensorTag";

static const uint16_t APP_ID = 1;
static esp_gatt_if_t app_gattc_if = ESP_GATT_IF_NONE;
static bool isTargetDetected = false;
static esp_bd_addr_t target_bdaddr = { 0 };

static bool get_localname_from_advdata(const uint8_t* advData, uint8_t advDataLength, char* buffer)
{
	for (const uint8_t* adStructure = advData; adStructure - advData < advDataLength; adStructure += adStructure[0]) {
		if (adStructure[0] == 0) break;

		uint8_t adType = adStructure[1];
		if (adType != 0x08 && adType != 0x09) continue;

		memcpy(buffer, adStructure + 2, adStructure[0] - 1);
		buffer[adStructure[0] - 1] = 0;
		return true;
	}

	buffer[0] = 0;
	return false;
}

static unique_ptr<GattClient> gatt_client;
static bool isFirstGapEvent = true;
static shared_ptr<GattClientService> temperature_service;
static const BluetoothUuid HumidityServiceUuid("F000AA20-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityDataCharacteristicUuid("F000AA21-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityConfigurationCharacteristicUuid("F000AA22-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityPeriodCharacteristicUuid("F000AA23-0451-4000-B000-000000000000");
static const BluetoothUuid NotificationDescriptorUuid(static_cast<uint16_t>(0x2902u), true);

static void handle_gap_event(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_SCAN_RESULT_EVT: {
		uint8_t* bda = param->scan_rst.bda;
		char localname[ESP_BLE_ADV_DATA_LEN_MAX];

		get_localname_from_advdata(param->scan_rst.ble_adv + param->scan_rst.adv_data_len, param->scan_rst.scan_rsp_len, localname);
		ESP_LOGI(TAG, "SCAN_RESULT: ADDR=%02x:%02x:%02x:%02x:%02x:%02x, NAME:%s, RSSI:%d", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], localname, param->scan_rst.rssi);
		if (strcmp(localname, TARGET_DEVICE_NAME) == 0) {
			ESP_LOGI(TAG, "Target device was detected, ADDR=%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
			ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
			memcpy(target_bdaddr, bda, sizeof(esp_bd_addr_t));
			gatt_client->set_discovery_completed_handler([](GattClient& client) {
				ESP_LOGI(TAG, "Discovery completed");
				temperature_service = client.get_service(HumidityServiceUuid);
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

static TlsClient tls_client;
static HttpClient http_client(tls_client);
static std::unique_ptr<freertos::WaitEvent> event_wifi_got_ip;

class ResponseReceiver : public IHttpResponseReceiver
{
private:
	bool is_success;

	virtual int on_message_begin(const HttpClient&) { ESP_LOGI(TAG, "Message begin"); return 0; }
	virtual int on_message_complete(const HttpClient&) { ESP_LOGI(TAG, "Message end"); return 0; }
	virtual int on_header_complete(const HttpClient&, const HttpResponseInfo& info) { 
		ESP_LOGI(TAG, "Header end. status code = %d", info.status_code); 
		ESP_LOGI(TAG, "Content-length = 0x%x", info.content_length);
		this->is_success = info.status_code == 200;	// Check status code to determinse the request has been completed successfully or not.
		return 0; 
	}
	virtual int on_header(const HttpClient&, const std::string& name, const std::string& value) {
		ESP_LOGI(TAG, "%s: %s", name.c_str(), value.c_str());
		return 0; 
	}
	virtual int on_body(const HttpClient&, const std::uint8_t* buffer, std::size_t length) {
		ESP_LOGI(TAG, "on_body: addr=%p, len=0x%x", buffer, length);
		return 0; 
	}
public:
	ResponseReceiver() : is_success(false) {}

	bool get_is_success() const { return this->is_success; }
};
esp_err_t wifi_event_handler(void* ctx, system_event_t* event) 
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_GOT_IP: {
		event_wifi_got_ip->set();
		break;
	}
	default:
		break;
	}
	return ESP_OK; 
}

static void initialize_wifi()
{
	tcpip_adapter_init();

	event_wifi_got_ip.reset(new freertos::WaitEvent());

	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, nullptr));
	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	wifi_config_t wifi_config = { 0 };
	strcpy(reinterpret_cast<char*>(wifi_config.sta.ssid), CONFIG_WIFI_SSID);
	strcpy(reinterpret_cast<char*>(wifi_config.sta.password), CONFIG_WIFI_PASSWORD);
	wifi_config.sta.bssid_set = false;
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_connect());

	event_wifi_got_ip->wait();
}

struct SensorData
{
	float temperature;
	float humidity;
};

freertos::WaitQueue<SensorData, 8> sensor_data_queue;

extern "C" void app_main();

void app_main()
{
	system_event_t hoge;

	static esp_ble_scan_params_t scan_params;

	ESP_ERROR_CHECK(nvs_flash_init());

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

	uint8_t cert;
	tls_client.initialize(&cert, 0);

	initialize_wifi();
	

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

	ESP_LOGI(TAG, "Enumerating humidity characteristics");
	auto result = temperature_service->enumerate_characteristics().get();
	ESP_LOGI(TAG, "Enumeration complete! result=%x", result);
	{
		ESP_LOGI(TAG, "Writing Configuration characteristic");
		auto characteristic = temperature_service->get_characteristic(HumidityConfigurationCharacteristicUuid);
		uint8_t value = 0x01;
		auto result = characteristic->write_value_async(&value, 1).get();
		ESP_LOGI(TAG, "Writing value complete. status=%x", result);
	}
	{
		ESP_LOGI(TAG, "Writing Period characteristic");
		auto characteristic = temperature_service->get_characteristic(HumidityPeriodCharacteristicUuid);
		uint8_t value = 200;
		auto result = characteristic->write_value_async(&value, 1).get();
		ESP_LOGI(TAG, "Writing value complete. status=%x", result);
	}
	{
		auto characteristic = temperature_service->get_characteristic(HumidityDataCharacteristicUuid);
		ESP_LOGI(TAG, "Enumerating descriptors...");
		auto result = characteristic->enumerate_descriptors().get();
		ESP_LOGI(TAG, "Enumerating descriptors completed. status=%x", result);

		characteristic->set_notification_handler([](const uint8_t* data, size_t length) {
			ESP_LOGI(TAG, "Notification. length=0x%x", length);
			if (length == 4) {
				int16_t raw_temp = static_cast<int16_t>(data[0] | (data[1] << 8));
				uint16_t raw_hum  = data[2] | (data[3] << 8);
				SensorData data;
				data.temperature = raw_temp*(165.0f / 65536.0f) - 40.0f;
				data.humidity = raw_hum / 65536.0f * 100.0f;

				ESP_LOGI(TAG, "handler: temp:%f, hum:%f", data.temperature, data.humidity);
				sensor_data_queue.send(data);
			}
		});
		result = characteristic->enable_notification_async(true).get();
		ESP_LOGI(TAG, "Notification enabled. status=%x", result);

		auto descriptor = characteristic->get_descriptor(NotificationDescriptorUuid);
		uint8_t value[2] = { 0x01, 0x00 };
		result = descriptor->write_value_async(value, sizeof(value)).get();
		ESP_LOGI(TAG, "Descriptor configured. status=%x", result);

		
		ResponseReceiver response_parser;
		unique_ptr<char> request_string(new char[2048]);
		while (true) {

			SensorData data;
			sensor_data_queue.receive(data);
			sprintf(request_string.get(), "%s&tmp=%f&hum=%f",
				"/test?name=ESP32", //"/api/HttpTriggerCSharp1?code=vICHv/UvPxv4SlwHcp78rPlKK8Cm6Fz47GpbwtpCbjoa2zpa/A3LRw==&name=ESP32",
				data.temperature,
				data.humidity);
			
			if (http_client.get("192.168.2.10", "443", request_string.get(), response_parser)) {
				if (!response_parser.get_is_success()) {
					ESP_LOGE(TAG, "Failed to send sensor data.");
				}
			}
		}
	}
}
