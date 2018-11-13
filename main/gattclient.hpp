#ifndef GATTCLIENT_HPP__
#define GATTCLIENT_HPP__

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>
#include <deque>
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

	std::deque<GattClientEventSink*> eventsinks;

	std::function<void(GattClient&)> discovery_completed_handler;

	void handle_gattc_event_inner(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

public:
	static void initialize() {
		shared_context = new (&shared_context_storage) SharedContext();
		ESP_ERROR_CHECK(esp_ble_gattc_register_callback(handle_gattc_event));
	}
	
	GattClient(uint16_t app_id) : gattc_app_id(app_id), gattc_if(ESP_GATT_IF_NONE), bd_addr(), conn_id(0), mtu(0), state(State::Closed), is_service_discovered(false)
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

	void register_eventsink(GattClientEventSink* eventsink)
	{
		ESP_LOGI(TAG, "EventSink %p registered.\n", eventsink);
		this->eventsinks.push_back(eventsink);
	}
	void unregister_eventsink(GattClientEventSink* eventsink)
	{
		ESP_LOGI(TAG, "EventSink %p unregistered.\n", eventsink);
		auto end = this->eventsinks.rend();
		for (auto it = this->eventsinks.rbegin(); it != end; ++it) {
			if (*it == eventsink) {
				this->eventsinks.erase(it.base());
				return;
			}
		}
	}
	template<typename THandler> void set_discovery_completed_handler(THandler&& handler) { this->discovery_completed_handler = std::forward<THandler>(handler); }
	template<typename THandler> void set_discovery_completed_handler(const THandler& handler) { this->discovery_completed_handler = handler; }

	GattClientService get_service(const BluetoothUuid& uuid, std::size_t index);

	bool open(const BdAddr& bd_addr, esp_ble_addr_type_t remote_addr_type) {
		this->bd_addr = bd_addr;
		this->state = State::Opening;
		auto result = esp_ble_gattc_open(this->gattc_if, this->bd_addr.value, remote_addr_type, true);
		if (result != ESP_OK) {
			ESP_LOGE(TAG, "Failed to open the device");
			this->state = State::Closed;
		}
		return result != ESP_OK;
	}
	void close() {
		esp_ble_gattc_close(this->gattc_if, this->conn_id);
	}

	bool is_opened() const { return this->state == State::Opened; }
};


#endif //GATTCLIENT_HPP__
