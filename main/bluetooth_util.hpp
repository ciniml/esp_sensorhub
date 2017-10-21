#ifndef BLUETOOTH_UTIL_HPP__
#define BLUETOOTH_UTIL_HPP__

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <functional>

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

struct BdAddr
{
	esp_bd_addr_t value;
	BdAddr() {}
	explicit BdAddr(const esp_bd_addr_t value) {
		memcpy(this->value, value, sizeof(esp_bd_addr_t));
	}
	bool operator==(const esp_bd_addr_t rhs) const { return std::memcmp(this->value, rhs, sizeof(esp_bd_addr_t)) == 0; }
	bool operator!=(const esp_bd_addr_t rhs) const { return !this->operator==(rhs); }

	bool operator==(const BdAddr& rhs) const { return this->operator==(rhs.value); }
	bool operator!=(const BdAddr& rhs) const { return this->operator!=(rhs.value); }
};

struct BluetoothUuid
{
	// Bluetooth Base UUID 00000000-0000-1000-8000-00805F9B34FB
	static const uint8_t base_uuid[16];
	esp_bt_uuid_t value;

	BluetoothUuid() : value() {}
	BluetoothUuid(const esp_bt_uuid_t value) : value(value) {}
	BluetoothUuid(uint16_t value, bool as_long_uuid) {
		if (as_long_uuid) {
			this->load_base_uuid();
			this->value.uuid.uuid128[12] = value & 0xff;
			this->value.uuid.uuid128[13] = value >> 8;
		}
		else {
			this->set_uuid16(value);
		}
	}
	BluetoothUuid(uint32_t value, bool as_long_uuid) {
		if (as_long_uuid) {
			this->load_base_uuid();
			this->value.uuid.uuid128[12] = (value >> 0) & 0xff;
			this->value.uuid.uuid128[13] = (value >> 8) & 0xff;
			this->value.uuid.uuid128[14] = (value >> 16) & 0xff;
			this->value.uuid.uuid128[15] = (value >> 24) & 0xff;
		}
		else {
			this->set_uuid32(value);
		}
	}
	BluetoothUuid(const char* uuid) {
		uint8_t octet = 0;

		for (size_t index = 31; *uuid; --index, ++uuid) {
			if (index == 23 || index == 19 || index == 15 || index == 11) {
				++uuid;
			}
			char c = *uuid;
			if (c == 0) break;
			uint8_t nibble = c < '0' ? 0 : c <= '9' ? (c - '0') : c < 'A' ? 0 : c <= 'F' ? c - 'A' + 10 : c < 'a' ? 0 : c <= 'f' ? c - 'a' + 10 : 0;
			octet = (octet << 4) | nibble;
			if ((index & 1) == 0) {
				this->value.uuid.uuid128[index >> 1] = octet;
			}
			if (index == 0) {
				this->value.len = ESP_UUID_LEN_128;
				return;
			}
		}
		this->value.len = 0;
		memset(this->value.uuid.uuid128, 0, 16);
	}

	void set_uuid16(uint16_t uuid16)
	{
		this->value.len = ESP_UUID_LEN_16;
		this->value.uuid.uuid16 = uuid16;
	}
	void set_uuid32(uint32_t uuid32)
	{
		this->value.len = ESP_UUID_LEN_32;
		this->value.uuid.uuid32 = uuid32;
	}
	void set_uuid(const uint8_t* uuid128)
	{
		this->value.len = ESP_UUID_LEN_128;
		memcpy(this->value.uuid.uuid128, uuid128, 16);
	}
	void load_base_uuid() { this->set_uuid(base_uuid); }
	BluetoothUuid to_uuid128() const
	{
		switch (this->value.len)
		{
		case ESP_UUID_LEN_16: return BluetoothUuid(this->value.uuid.uuid16, true);
		case ESP_UUID_LEN_32: return BluetoothUuid(this->value.uuid.uuid32, true);
		case ESP_UUID_LEN_128: return *this;
		default: return BluetoothUuid();
		}
	}
	bool operator==(const BluetoothUuid& rhs) const {
		if (this->value.len == rhs.value.len) {
			switch (this->value.len)
			{
			case ESP_UUID_LEN_16: return this->value.uuid.uuid16 == rhs.value.uuid.uuid16;
			case ESP_UUID_LEN_32: return this->value.uuid.uuid32 == rhs.value.uuid.uuid32;
			case ESP_UUID_LEN_128: return memcmp(this->value.uuid.uuid128, rhs.value.uuid.uuid128, 16) == 0;
			default: return false;
			}
		}
		else {
			return this->to_uuid128() == rhs.to_uuid128();
		}
	}
	bool operator!=(const BluetoothUuid& rhs) const { return !this->operator==(rhs); }
	bool operator<(const BluetoothUuid& rhs) const {
		if (this->value.len == rhs.value.len) {
			switch (this->value.len)
			{
			case ESP_UUID_LEN_16: return this->value.uuid.uuid16 < rhs.value.uuid.uuid16;
			case ESP_UUID_LEN_32: return this->value.uuid.uuid32 < rhs.value.uuid.uuid32;
			case ESP_UUID_LEN_128: return memcmp(this->value.uuid.uuid128, rhs.value.uuid.uuid128, 16) < 0;
			default: return false;
			}
		}
		else {
			return this->to_uuid128() < rhs.to_uuid128();
		}
	}
	void to_string(char* buffer) const {
		const uint8_t* uuid128 = this->value.uuid.uuid128;

		switch (this->value.len)
		{
		case ESP_UUID_LEN_16: {
			sprintf(buffer, "0x%04x", this->value.uuid.uuid16);
			break;
		}
		case ESP_UUID_LEN_32: {
			sprintf(buffer, "0x%08x", this->value.uuid.uuid32);
			break;
		}
		case ESP_UUID_LEN_128: {
			uint32_t a = (uuid128[15] << 24) | (uuid128[14] << 16) | (uuid128[13] << 8) | (uuid128[12]);
			uint16_t b = (uuid128[11] << 8) | (uuid128[10]);
			uint16_t c = (uuid128[9] << 8) | (uuid128[8]);
			uint16_t d = (uuid128[7] << 8) | (uuid128[6]);
			uint16_t e = (uuid128[5] << 8) | (uuid128[4]);
			uint32_t f = (uuid128[3] << 24) | (uuid128[2] << 16) | (uuid128[1] << 8) | (uuid128[0]);
			sprintf(buffer, "{%08x-%04x-%04x-%04x-%04x%08x}", a, b, c, d, e, f);
			break;
		}
		default: strcpy(buffer, "{invalid}");
		}

	}
	std::string str() const {
		char buffer[40];
		this->to_string(buffer);
		return std::string(buffer);
	}
};

namespace std
{
	template<> struct less<esp_bt_uuid_t> {
		bool operator()(const esp_bt_uuid_t& lhs, const esp_bt_uuid_t& rhs) {
			return memcmp(&lhs, &rhs, sizeof(esp_bt_uuid_t)) < 0;
		}
	};
};

struct GattClientEventSink
{
	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) = 0;
};

class GattClientService;

class GattClient
{
public:
	enum class State
	{
		Closed,
		Opening,
		Opened,
		Connected,
	};

	static void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

private:
	struct SharedContext
	{
		freertos::Mutex clients_mutex;
		std::vector<std::shared_ptr<GattClient>> clients;
	};

	static std::aligned_storage<sizeof(SharedContext), alignof(SharedContext)>::type shared_context_storage;
	static SharedContext* shared_context;

	static const char TAG[];
	
	uint16_t gattc_app_id;
	esp_gatt_if_t gattc_if;
	BdAddr bd_addr;
	uint16_t conn_id;
	uint16_t mtu;
	State state;
	bool is_service_discovered;
	std::map<BluetoothUuid, esp_gatt_srvc_id_t> services;
	std::vector<std::weak_ptr<GattClientEventSink>> eventsinks;

	std::function<void(GattClient&)> discovery_completed_handler;

	void begin_update_services() {
		ESP_ERROR_CHECK(esp_ble_gattc_search_service(this->gattc_if, this->conn_id, nullptr));
	}

	void handle_gattc_event_inner(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
	{
		switch (event)
		{
		case ESP_GATTC_OPEN_EVT: {
			if (param->open.status == ESP_GATT_OK) {
				this->conn_id = param->open.conn_id;
				this->mtu = param->open.mtu;
				this->state = State::Opened;
				ESP_LOGI(TAG, "Device opened. conn_id=%x, mtu=%x", this->conn_id, this->mtu);
			}
			else {
				ESP_LOGE(TAG, "Failed to open. status=%x", param->open.status);
			}
			break;
		}
		case ESP_GATTC_CLOSE_EVT: {
			if (this->bd_addr == param->close.remote_bda) {
				this->state = State::Closed;
				ESP_LOGI(TAG, "Device closed.");
			}
			break;
		}
		case ESP_GATTC_CONNECT_EVT: {
			if (this->bd_addr == param->connect.remote_bda) {
				if (param->connect.status == ESP_GATT_OK) {
					this->state = State::Connected;
					this->services.clear();
					this->begin_update_services();
					ESP_LOGI(TAG, "Device connected.");
				}
			}
			break;
		}
		case ESP_GATTC_SRVC_CHG_EVT: {
			if (this->bd_addr == param->srvc_chg.remote_bda) {
				this->services.clear();
				this->begin_update_services();
				ESP_LOGI(TAG, "Service changed.");
			}
			break;
		}
		case ESP_GATTC_SEARCH_RES_EVT: {    // Service discovery result.
			if (param->search_res.conn_id == this->conn_id) {
				BluetoothUuid uuid(param->search_res.srvc_id.id.uuid);
				char buffer[128];
				uuid.to_string(buffer);
				ESP_LOGI(TAG, "Service discovered. UUID=%s", buffer);
				this->services.insert(std::make_pair(param->search_res.srvc_id.id.uuid, param->search_res.srvc_id));
			}
			break;
		}
		case ESP_GATTC_SEARCH_CMPL_EVT: {
			if (param->search_cmpl.conn_id == this->conn_id) {
				ESP_LOGI(TAG, "Service discovery completed.");
				if (param->search_cmpl.status == ESP_GATT_OK) {
					this->is_service_discovered = true;
					if (this->discovery_completed_handler) {
						this->discovery_completed_handler(*this);
					}
				}
				else {
					this->is_service_discovered = false;
				}
			}
			break;
		}
		default: break;
		}

		for (auto& ptr : this->eventsinks) {
			if (!ptr.expired()) {
				auto sink = ptr.lock();
				sink->handle_gattc_event(event, gattc_if, param);
			}
		}
	}

public:
	static void initialize() {
		shared_context = new (&shared_context_storage) SharedContext();
		ESP_ERROR_CHECK(esp_ble_gattc_register_callback(handle_gattc_event));
	}
	
	GattClient(uint16_t app_id) : gattc_app_id(app_id), gattc_if(ESP_GATT_IF_NONE), bd_addr(), conn_id(0), mtu(0), state(State::Closed), is_service_discovered(false), services()
	{
		if (shared_context == nullptr) {
			initialize();
		}
		freertos::LockGuard<freertos::Mutex> guard(shared_context->clients_mutex);
		shared_context->clients.push_back(std::shared_ptr<GattClient>(this));
		ESP_ERROR_CHECK(esp_ble_gattc_app_register(app_id));
	}
	BdAddr get_bd_addr() const { return this->bd_addr; }

	uint16_t get_connection_id() const { return this->conn_id; }
	esp_gatt_if_t get_gattc_if() const { return this->gattc_if; }

	void register_eventsink(std::shared_ptr<GattClientEventSink> eventsink)
	{
		this->eventsinks.push_back(eventsink);
	}

	template<typename THandler> void set_discovery_completed_handler(THandler&& handler) { this->discovery_completed_handler = std::forward<THandler>(handler); }
	template<typename THandler> void set_discovery_completed_handler(const THandler& handler) { this->discovery_completed_handler = handler; }

	std::shared_ptr<GattClientService> get_service(const BluetoothUuid& uuid);

	bool open(const BdAddr& bd_addr) {
		this->bd_addr = bd_addr;
		this->state = State::Opening;
		auto result = esp_ble_gattc_open(this->gattc_if, this->bd_addr.value, true);
		if (result != ESP_OK) {
			ESP_LOGE(TAG, "Failed to open the device");
			this->state = State::Closed;
		}
		return result != ESP_OK;
	}
	void close() {
		esp_ble_gattc_close(this->gattc_if, this->conn_id);
	}
};


class GattClientCharacteristic;

class GattClientService : public GattClientEventSink
{
private:
	GattClient& client;
	esp_gatt_srvc_id_t id;

	freertos::single_promise<esp_err_t> enumerate_promise;
	std::map<BluetoothUuid, std::shared_ptr<GattClientCharacteristic>> characteristics;

	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
public:
	GattClientService(GattClient& client, const esp_gatt_srvc_id_t& id) : client(client), id(id) {}
	GattClient& get_client() { return this->client; }
	esp_gatt_srvc_id_t get_id() const { return this->id; }

	freertos::future<esp_err_t> enumerate_characteristics();
	std::shared_ptr<GattClientCharacteristic> get_characteristic(const BluetoothUuid uuid) const;
};

class GattClientDescriptor;

class GattClientCharacteristic : public GattClientEventSink
{
	friend class GattClientService;
	friend class GattClientDescriptor;
private:
	static const char TAG[];

	GattClientService& service;
	esp_gatt_id_t id;

	uint16_t get_connection_id() const { return this->service.get_client().get_connection_id(); }
	esp_gatt_if_t get_gattc_if() const { return this->service.get_client().get_gattc_if(); }

	bool is_descriptor_enumeratred;
	std::map<BluetoothUuid, std::shared_ptr<GattClientDescriptor>> descriptors;
	freertos::single_promise<esp_err_t> enumerate_descriptors_promise;

	std::function<void(const uint8_t* data, size_t length)> notification_handler;
	std::function<void(esp_gatt_status_t, uint8_t*, size_t)> value_read_handler;
	
	freertos::single_promise<esp_err_t> value_write_promise;

	freertos::single_promise<esp_err_t> enable_notification_promise;
	

	void update_id(esp_gatt_id_t new_id)
	{
		this->id = new_id;
	}
public:
	GattClientCharacteristic(GattClientService& service, esp_gatt_id_t id);


	esp_gatt_id_t get_id() const { return this->id; }

	freertos::future<esp_err_t> enumerate_descriptors();
	std::shared_ptr<GattClientDescriptor> get_descriptor(const BluetoothUuid uuid) const;

	template<typename THandler>
	void set_value_read_handler(THandler&& handler) { this->value_read_handler = std::forward<THandler>(handler); }
	template<typename THandler>
	void set_value_read_handler(const THandler& handler) { this->value_read_handler = handler; }

	esp_err_t begin_read_value();
	freertos::future<esp_err_t> write_value_async(const uint8_t* buffer, size_t length);
	freertos::future<esp_err_t> enable_notification_async(bool enable);

	template<typename THandler>
	void set_notification_handler(THandler&& handler) { this->notification_handler = std::forward<THandler>(handler); }
	template<typename THandler>
	void set_notification_handler(const THandler& handler) { this->notification_handler = handler; }

	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
};

class GattClientDescriptor : public GattClientEventSink
{
private:
	friend class GattClientCharacteristic;

	GattClientCharacteristic& characteristic;
	esp_gatt_id_t id;

	uint16_t get_connection_id() const { return this->characteristic.get_connection_id(); }
	esp_gatt_if_t get_gattc_if() const { return this->characteristic.get_gattc_if(); }


	struct WriteContext
	{
		std::shared_ptr<uint8_t> data;
		size_t length;
	} write_context;
	freertos::single_promise<esp_err_t> write_promise;

public:
	GattClientDescriptor(GattClientCharacteristic& characteristic, esp_gatt_id_t id);

	esp_gatt_id_t get_id() const { return this->id; }
	
	freertos::future<esp_err_t> write_value_async(const uint8_t* buffer, size_t length);
	
	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);
};

#endif //BLUETOOTH_UTIL_HPP__
