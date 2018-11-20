#ifndef GATTCLIENTCHARACTERISTIC_HPP__
#define GATTCLIENTCHARACTERISTIC_HPP__

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

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"

#include "freertos_future.hpp"

#include "bluetooth_util.hpp"

class GattClient;
class GattClientDescriptor;

class GattClientCharacteristic : public GattClientEventSink
{
	friend class GattClientService;
	friend class GattClientDescriptor;
private:
	static const char TAG[];

	GattClient* client;
	esp_gattc_char_elem_t element;
	

	uint16_t get_connection_id() const;
	esp_gatt_if_t get_gattc_if() const;

	bool is_descriptor_enumeratred;
	std::map<BluetoothUuid, std::shared_ptr<GattClientDescriptor>> descriptors;
	freertos::single_promise<esp_err_t> enumerate_descriptors_promise;

	std::function<void(const uint8_t* data, std::size_t length)> notification_handler;
	std::function<void(esp_gatt_status_t, const uint8_t*, std::size_t)> value_read_handler;

	freertos::single_promise<esp_err_t> value_write_promise;
	freertos::single_promise<esp_err_t> enable_notification_promise;
public:
	GattClientCharacteristic() : client(nullptr) {}
	GattClientCharacteristic(GattClient& client, const esp_gattc_char_elem_t& element);
	GattClientCharacteristic(const GattClientCharacteristic& obj);
	GattClientCharacteristic(GattClientCharacteristic&&);
	~GattClientCharacteristic();

	BluetoothUuid get_id() const { return BluetoothUuid(this->element.uuid); }
	
	GattClientDescriptor get_descriptor(const BluetoothUuid uuid);

	template<typename THandler>
	void set_value_read_handler(THandler&& handler) { this->value_read_handler = std::forward<THandler>(handler); }
	template<typename THandler>
	void set_value_read_handler(const THandler& handler) { this->value_read_handler = handler; }

	esp_err_t begin_read_value();
	freertos::future<esp_err_t> write_value_async(const uint8_t* buffer, std::size_t length);
	freertos::future<esp_err_t> enable_notification_async(bool enable);

	template<typename THandler>
	void set_notification_handler(THandler&& handler) { this->notification_handler = std::forward<THandler>(handler); }
	template<typename THandler>
	void set_notification_handler(const THandler& handler) { this->notification_handler = handler; }

	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

	GattClientCharacteristic& operator=(const GattClientCharacteristic& rhs);
	operator bool() const { return this->client != nullptr; }
};

class GattClientDescriptor : public GattClientEventSink
{
private:
	friend class GattClientCharacteristic;

	GattClient* client;
	BluetoothUuid uuid;
	esp_gattc_descr_elem_t element;

	uint16_t get_connection_id() const;
	esp_gatt_if_t get_gattc_if() const;


	struct WriteContext
	{
		std::shared_ptr<uint8_t> data;
		std::size_t length;
	} write_context;
	freertos::single_promise<esp_err_t> write_promise;

public:
	GattClientDescriptor() : client(nullptr) {}
	GattClientDescriptor(GattClient& client, const esp_gattc_descr_elem_t& element);
	GattClientDescriptor(GattClientDescriptor&&);
	~GattClientDescriptor();

	BluetoothUuid get_uuid() const { return this->uuid; }
	
	freertos::future<esp_err_t> write_value_async(const uint8_t* buffer, std::size_t length);
	
	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);

	GattClientDescriptor& operator=(const GattClientDescriptor& rhs);
	operator bool() const { return this->client != nullptr; }
};

#endif //GATTCLIENTCHARACTERISTIC_HPP__
