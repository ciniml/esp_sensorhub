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

class GattClientException : std::runtime_error
{
private:
	esp_err_t error;
public:
	GattClientException(esp_err_t error) : std::runtime_error("GattClientException"), error(error) {}
	esp_err_t get_error() const { return this->error; }
};

#endif //BLUETOOTH_UTIL_HPP__
