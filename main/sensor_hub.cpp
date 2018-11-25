#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <thread>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#include <esp_wifi.h>

#include "freertos_future.hpp"
#include "bluetooth_util.hpp"
#include "gattclient.hpp"
#include "gattclientservice.hpp"
#include "gattclientcharacteristic.hpp"
#include "lazy.hpp"
#include "string_utils.hpp"

#include "http_client.hpp"

static const BluetoothUuid target_uuid("F0000000-0451-4000-B000-000000000000");

using namespace std;
static const char* TAG = "SENSORHUB";
static const char TARGET_DEVICE_NAME[] = "CC2650 SensorTag";

//SORACOM Harvestに接続するための証明書 (main/component.mkのCOMPONENT_EMBED_TXTFILESで指定)
extern const char api_soracom_io_crt_start[] asm("_binary_api_soracom_io_pem_start");
extern const char api_soracom_io_crt_end[] asm("_binary_api_soracom_io_pem_end");
//SORACOM Inventryで登録したデバイス情報
extern const char inventry_device_start[] asm("_binary_device_txt_start");
extern const char inventry_device_end[] asm("_binary_device_txt_end");

//
static HttpClient http_client(api_soracom_io_crt_start);		// HTTP通信用インスタンス

static std::string makeSoracomHarvestEndpointUrl(const char* deviceId, const char* deviceSecret)
{
	static const ConstantString part0("https://api.soracom.io/v1/devices/");
	static const ConstantString part1("/publish?device_secret=");
	
	return concatStrings(part0, deviceId, part1, deviceSecret);
}
static std::string getInventryDeviceId()
{
	std::string result;
	result.reserve(32);
	for(const char* p = inventry_device_start; p < inventry_device_end; p++) {
		if( *p == ',' ) {
			break;
		}
		else {
			result.push_back(*p);
		}
	}
	return result;
}
static std::string getInventryDeviceSecret()
{
	int column = 0;
	std::string result;
	result.reserve(32);
	for(const char* p = inventry_device_start; p < inventry_device_end; p++) {
		if( *p == ',' ) {
			column++;
		}
		else if( column == 2 ) {
			result.push_back(*p);
		}
	}
	return result;
}


// HTTPレスポンスを解析するIHttpResponseReceiverの実装
class ResponseReceiver : public IHttpResponseReceiver
{
private:

	virtual int on_header(const HttpClient&, const std::string& name, const std::string& value) {
		ESP_LOGI(TAG, "%s: %s", name.c_str(), value.c_str());
		return 0;
	}
	virtual int on_body(const HttpClient&, const std::uint8_t* buffer, std::size_t length) {
		//ESP_LOGI(TAG, "on_body: addr=%p, len=0x%x", buffer, length);
		return 0;
	}
public:
	ResponseReceiver() {}
};

static const uint16_t APP_ID = 1;
static esp_gatt_if_t app_gattc_if = ESP_GATT_IF_NONE;
static bool isTargetDetected = false;
static esp_bd_addr_t target_bdaddr = { 0 };

static const char* NVS_NS_DEVICE = "device";
static const char* NVS_KEY_DEVICE_ADDRESS = "address";

/**
 * @brief 接続対象のデバイス・アドレスをNVSから読み込む。
 * @return アドレスが保存されていて読み込みが成功すればtrue。それ以外の場合はfalse。
 */
static bool load_target_device_address()
{
	nvs_handle handle;
	auto err = nvs_open(NVS_NS_DEVICE, NVS_READWRITE, &handle);
	if( err != ESP_OK ) {
		ESP_LOGW(TAG, "Failed to open NVS namespace.");
		return false;
	}
	size_t size = sizeof(esp_bd_addr_t);
	uint8_t bda[sizeof(esp_bd_addr_t)];
	err = nvs_get_blob(handle, NVS_KEY_DEVICE_ADDRESS, bda, &size);
	nvs_close(handle);
	if( err != ESP_OK ) {
		ESP_LOGE(TAG, "No device address key were found.");
		return false;
	}
	else if( size != sizeof(esp_bd_addr_t) ) {
		ESP_LOGE(TAG, "Wrong device address size, size=%zu", size);
		return false;
	}
	ESP_LOGI(TAG, "Using stored device address: %02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
	memcpy(target_bdaddr, bda, sizeof(esp_bd_addr_t));
	// 読み込んだアドレスがすべて0でないかどうかチェックする。
	for(std::size_t i = 0; i < sizeof(esp_bd_addr_t); i++) {
		if( bda[i] != 0 ) {
			return true;
		}
	}
	return false;
}
/**
 * @brief 現在のデバイス・アドレスをNVSに保存する
 * @return 保存処理が成功したらtrue。失敗したらfalse。
 */
static bool store_target_device_address()
{
	nvs_handle handle;
	auto err = nvs_open(NVS_NS_DEVICE, NVS_READWRITE, &handle);
	if( err != ESP_OK ) {
		ESP_LOGW(TAG, "Failed to open NVS namespace.");
		return false;
	}
	uint8_t* bda = reinterpret_cast<uint8_t*>(&target_bdaddr);
	ESP_LOGI(TAG, "Using stored device address: %02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
	err = nvs_set_blob(handle, NVS_KEY_DEVICE_ADDRESS, target_bdaddr, sizeof(esp_bd_addr_t));
	if( err != ESP_OK ) {
		nvs_close(handle);
		ESP_LOGE(TAG, "Failed to store device address.");
		return false;
	}
	err = nvs_commit(handle);
	nvs_close(handle);
	if( err != ESP_OK ) {
		ESP_LOGE(TAG, "Failed to commit NVS change.");
		return false;
	}
	return true;
}
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
static GattClientService temperature_service;
static GattClientService battery_service;
static const BluetoothUuid BatteryServiceUuid(static_cast<uint16_t>(0x180f), true);
static const BluetoothUuid BatteryLevelCharacteristicUuid(static_cast<uint16_t>(0x2a19), true);

static const BluetoothUuid HumidityServiceUuid("F000AA20-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityDataCharacteristicUuid("F000AA21-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityConfigurationCharacteristicUuid("F000AA22-0451-4000-B000-000000000000");
static const BluetoothUuid HumidityPeriodCharacteristicUuid("F000AA23-0451-4000-B000-000000000000");
static const BluetoothUuid NotificationDescriptorUuid(static_cast<uint16_t>(0x2902u), true);

static void connect_device(bool rescan_when_fail)
{
	gatt_client->set_discovery_completed_handler([](GattClient& client) {
		temperature_service = gatt_client->get_service(HumidityServiceUuid, 0);
		if(!temperature_service) {
			ESP_LOGE(TAG, "Failed to get temperature service.");
		}
		battery_service = gatt_client->get_service(BatteryServiceUuid, 0);
		if(!battery_service) {
			ESP_LOGE(TAG, "Failed to get battery service.");
		}
		ESP_LOGI(TAG, "Discovery completed");
	});
	while( !gatt_client->open(BdAddr(target_bdaddr), BLE_ADDR_TYPE_PUBLIC) ) {
		if( rescan_when_fail ) {
			// デバイスを開けなかったので再度スキャンする
			ESP_ERROR_CHECK(esp_ble_gap_start_scanning(60));   // Scan 60[s]
			break;
		}
		else {
			// デバイスを開けなかったので30秒待って再度接続する。
			ESP_LOGE(TAG, "Failed to connect device. retry after 30[s].");
			vTaskDelay(pdMS_TO_TICKS(30*1000));
		}
	}
}
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
			connect_device(true);
		}
		break;
	}
	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
		ESP_LOGI(TAG, "BLE Scan started. status=%x", param->scan_start_cmpl.status);
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

static void main_task(void* param) {
	ESP_LOGI(TAG, "Initializing ESP Bluedroid...");

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK);
	ESP_ERROR_CHECK(esp_bluedroid_init());
	ESP_ERROR_CHECK(esp_bluedroid_enable());

	// Initialize GattClient
	GattClient::initialize();

	gatt_client.reset(new GattClient(1));

	bool is_device_address_stored = load_target_device_address();
	if( is_device_address_stored ) {
		// 接続先デバイス・アドレスが保存されているので、接続する。
		connect_device(false);
	}
	else {
		// スキャンする。
		memset(target_bdaddr, 0, sizeof(esp_bd_addr_t));
		static esp_ble_scan_params_t scan_params;
		ESP_LOGI(TAG, "Initialzing ESP BLE...");
		ESP_ERROR_CHECK(esp_ble_gap_register_callback(handle_gap_event));
		scan_params.scan_type = BLE_SCAN_TYPE_ACTIVE;
		scan_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
		scan_params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
		scan_params.scan_interval = 1600;   // 1[s]
	 	scan_params.scan_window = 320;      // 200[ms]

		ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
		ESP_ERROR_CHECK(esp_ble_gap_start_scanning(60));   // Scan 60[s]
	}
	
	
	// サービスの検出待ち
	while (!temperature_service && !battery_service) { vTaskDelay(portTICK_PERIOD_MS); }

	// 接続先デバイス・アドレスが保存されていなければ保存する。
	if( !is_device_address_stored ) {
		store_target_device_address();
	}

	{
		ESP_LOGI(TAG, "Writing Configuration characteristic");
		auto characteristic = temperature_service.get_characteristic(HumidityConfigurationCharacteristicUuid);
		uint8_t value = 0x01;
		auto result = characteristic.write_value_async(&value, 1).get();
		ESP_LOGI(TAG, "Writing value complete. status=%x", result);
	}
	{
		ESP_LOGI(TAG, "Writing Period characteristic");
		auto characteristic = temperature_service.get_characteristic(HumidityPeriodCharacteristicUuid);
		uint8_t value = 200;	// 200*10[ms]
		auto result = characteristic.write_value_async(&value, 1).get();
		ESP_LOGI(TAG, "Writing value complete. status=%x", result);
	}

	volatile uint8_t battery_level = 0;
	GattClientCharacteristic battery_characteristic = battery_service.get_characteristic(BatteryLevelCharacteristicUuid);
	if( battery_service ) {
		ESP_LOGI(TAG, "Enable battery level notification.");
		battery_characteristic.set_notification_handler([&battery_level](const uint8_t* data, std::size_t length) {
			if( length > 0 ) {
				battery_level = *data;
			}
		});
		battery_characteristic.set_value_read_handler([&battery_level](esp_gatt_status_t status, const uint8_t* data, std::size_t length) {
			if( length > 0 ) {
				battery_level = *data;
			}
		});
		auto result = battery_characteristic.enable_notification_async(true).get();
		ESP_LOGI(TAG, "Notification enabled. status=%x", result);
		auto descriptor = battery_characteristic.get_descriptor(NotificationDescriptorUuid);
		uint8_t value[2] = {0x01, 0x00};
		result = descriptor.write_value_async(value, sizeof(value)).get();
		ESP_LOGI(TAG, "Descriptor configured. status=%x", result);

		battery_characteristic.begin_read_value();
	}

	GattClientCharacteristic temperature_characteristic = temperature_service.get_characteristic(HumidityDataCharacteristicUuid);
	{
		sensor_data_queue.ensure();
		
		temperature_characteristic.set_notification_handler([](const uint8_t* data, std::size_t length) {
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
		auto result = temperature_characteristic.enable_notification_async(true).get();
		ESP_LOGI(TAG, "Notification enabled. status=%x", result);

		auto descriptor = temperature_characteristic.get_descriptor(NotificationDescriptorUuid);
		uint8_t value[2] = { 0x01, 0x00 };
		result = descriptor.write_value_async(value, sizeof(value)).get();
		ESP_LOGI(TAG, "Descriptor configured. status=%x", result);
	}
	{

		vector<char> payload;
		payload.reserve(256);

		auto inventryDeviceId = getInventryDeviceId();
		auto inventryDeviceSecret = getInventryDeviceSecret();
		auto harvestEndpoint = makeSoracomHarvestEndpointUrl(inventryDeviceId.c_str(), inventryDeviceSecret.c_str());

		sensor_data_queue->reset();
		while (true) {
			const int averageSamples = 30;
			SensorData data;
			SensorData average = {0};
			for(int i = 0; i < averageSamples; i++ ) {
				sensor_data_queue->receive(data);
				average.temperature += data.temperature;
				average.humidity    += data.humidity;
			}
			average.temperature /= averageSamples;
			average.humidity    /= averageSamples;

			ResponseReceiver response_parser;
			HttpResponseInfo response;
			std::size_t payload_length = sprintf(payload.data(), 
				"{\"tmp\":%f,\"hum\":%f,\"bat\":%d}",
				average.temperature,
				average.humidity,
				battery_level);
			if (http_client.perform(HTTP_METHOD_POST, harvestEndpoint.c_str(), "application/json", payload.data(), payload_length, response_parser, response)) {
				if (response.status_code < 200 || response.status_code >= 300) {
					ESP_LOGE(TAG, "Failed to send sensor data. status_code=%03d", response.status_code);
				}
			}
		}
	}
}

extern "C" void app_main();

void app_main()
{
	auto err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
		// ページ不足の場合はNVSをクリアする。
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	ESP_ERROR_CHECK(err);

	// Initialize WiFi connection
	initialize_wifi();
	
	xTaskCreatePinnedToCore(&main_task, "main_task", 36*1024, NULL, 5, NULL, PRO_CPU_NUM);
}
