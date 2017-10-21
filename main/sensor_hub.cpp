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

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"


#include "freertos_future.hpp"
#include "bluetooth_util.hpp"
#include "lazy.hpp"

//AWS IoTÇ…ê⁄ë±Ç∑ÇÈÇΩÇﬂÇÃèÿñæèë ( main/component.mkÇÃCOMPONENT_EMBED_TXTFILESÇ≈éwíË)
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

static char aws_iot_host_address[] = AWS_IOT_MQTT_HOST;
static constexpr uint32_t aws_iot_port = AWS_IOT_MQTT_PORT;

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

static std::unique_ptr<freertos::WaitEvent> event_wifi_got_ip;

static esp_err_t wifi_event_handler(void* ctx, system_event_t* event)
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

Lazy<freertos::WaitQueue<SensorData, 8>> sensor_data_queue;

static void aws_iot_disconnect_handler(AWS_IoT_Client *pClient, void *data) {
	ESP_LOGW(TAG, "MQTT Disconnect");
	IoT_Error_t rc = FAILURE;

	if (NULL == pClient) {
		return;
	}

	if (aws_iot_is_autoreconnect_enabled(pClient)) {
		ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
	}
	else {
		ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
		rc = aws_iot_mqtt_attempt_reconnect(pClient);
		if (NETWORK_RECONNECTED == rc) {
			ESP_LOGW(TAG, "Manual Reconnect Successful");
		}
		else {
			ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
		}
	}
}
static void aws_iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
	IoT_Publish_Message_Params *params, void *pData) {
	ESP_LOGI(TAG, "Subscribe callback");
	ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int)params->payloadLen, (char *)params->payload);
}

static void initialize_aws_iot(AWS_IoT_Client& client)
{
	int32_t i = 0;

	IoT_Error_t rc = FAILURE;
	
	IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
	IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

	mqttInitParams.enableAutoReconnect = false;
	mqttInitParams.pHostURL = aws_iot_host_address;
	mqttInitParams.port = aws_iot_port;

	mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
	mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
	mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

	mqttInitParams.mqttCommandTimeout_ms = 20000;
	mqttInitParams.tlsHandshakeTimeout_ms = 5000;
	mqttInitParams.isSSLHostnameVerify = true;
	mqttInitParams.disconnectHandler = aws_iot_disconnect_handler;
	mqttInitParams.disconnectHandlerData = NULL;

	rc = aws_iot_mqtt_init(&client, &mqttInitParams);
	if (SUCCESS != rc) {
		ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
		abort();
	}

	connectParams.keepAliveIntervalInSec = 10;
	connectParams.isCleanSession = true;
	connectParams.MQTTVersion = MQTT_3_1_1;
	connectParams.pClientID = CONFIG_AWS_CLIENT_ID;
	connectParams.clientIDLen = (uint16_t)strlen(CONFIG_AWS_CLIENT_ID);
	connectParams.isWillMsgPresent = false;

	ESP_LOGI(TAG, "Connecting to AWS...");
	do {
		rc = aws_iot_mqtt_connect(&client, &connectParams);
		if (rc != SUCCESS) {
			ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	} while (rc != SUCCESS);

	rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
	if (SUCCESS != rc) {
		ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
		abort();
	}
	ESP_LOGI(TAG, "Connected to AWS");
}

static AWS_IoT_Client aws_iot_client;
static volatile uint32_t super_buffer[128];

static void aws_iot_task(void* param) {
	
	
	// initialize AWS IoT connection
	initialize_aws_iot(aws_iot_client);

	ESP_LOGI(TAG, "Initializing ESP Bluedroid...");

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK);
	ESP_ERROR_CHECK(esp_bluedroid_init());
	ESP_ERROR_CHECK(esp_bluedroid_enable());

	// Initialize GattClient
	GattClient::initialize();


	static esp_ble_scan_params_t scan_params;
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
		sensor_data_queue.ensure();

		auto characteristic = temperature_service->get_characteristic(HumidityDataCharacteristicUuid);
		ESP_LOGI(TAG, "Enumerating descriptors...");
		auto result = characteristic->enumerate_descriptors().get();
		ESP_LOGI(TAG, "Enumerating descriptors completed. status=%x", result);

		characteristic->set_notification_handler([](const uint8_t* data, size_t length) {
			ESP_LOGI(TAG, "Notification. length=0x%x", length);
			if (length == 4) {
				int16_t raw_temp = static_cast<int16_t>(data[0] | (data[1] << 8));
				uint16_t raw_hum = data[2] | (data[3] << 8);
				SensorData data;
				data.temperature = raw_temp*(165.0f / 65536.0f) - 40.0f;
				data.humidity = raw_hum / 65536.0f * 100.0f;

				ESP_LOGI(TAG, "handler: temp:%f, hum:%f", data.temperature, data.humidity);
				sensor_data_queue->send(data);
			}
		});
		result = characteristic->enable_notification_async(true).get();
		ESP_LOGI(TAG, "Notification enabled. status=%x", result);

		auto descriptor = characteristic->get_descriptor(NotificationDescriptorUuid);
		uint8_t value[2] = { 0x01, 0x00 };
		result = descriptor->write_value_async(value, sizeof(value)).get();
		ESP_LOGI(TAG, "Descriptor configured. status=%x", result);


		unique_ptr<char> payload(new char[256]);
		while (true) {
			IoT_Error_t rc;

			ESP_LOGI(TAG, "process AWS IoT MQTT");
			rc = aws_iot_mqtt_yield(&aws_iot_client, 100);
			if (NETWORK_ATTEMPTING_RECONNECT == rc) {
				// If the client is attempting to reconnect we will skip the rest of the loop.
				sensor_data_queue->reset();
				vTaskDelay(pdMS_TO_TICKS(1));
				continue;
			}

			SensorData data;
			sensor_data_queue->receive(data);

			const char *TOPIC = "test_topic/esp32";
			const int TOPIC_LEN = strlen(TOPIC);
			IoT_Publish_Message_Params params;

			params.qos = QOS0;
			params.payload = payload.get();
			params.isRetained = 0;
			params.payloadLen = sprintf(payload.get(), "{\"temp\":%f, \"hum\":%f}", data.temperature, data.humidity);
			ESP_LOGI(TAG, "Publishing a message: %s", payload.get());
			rc = aws_iot_mqtt_publish(&aws_iot_client, TOPIC, TOPIC_LEN, &params);

			if (rc != SUCCESS) {
				ESP_LOGE(TAG, "Failed to publish(rc=%d)", rc);
			}
		}
	}
}

extern "C" void app_main();

void app_main()
{
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

	// Initialize WiFi connection
	initialize_wifi();
	
	xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 36*1024, NULL, 5, NULL, 0);
}
