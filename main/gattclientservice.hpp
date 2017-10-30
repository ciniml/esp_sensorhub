#ifndef GATTCLIENTSERVICE_HPP__
#define GATTCLIENTSERVICE_HPP__

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

#include "bluetooth_util.hpp"

class GattClient;
class GattClientDescriptor;
class GattClientCharacteristic;

class GattClientService : public GattClientEventSink
{
	friend class GattClient;
private:
	static const char TAG[];

	GattClient* client;
	BluetoothUuid uuid;
	uint16_t start_handle;
	uint16_t end_handle;

	virtual void handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
public:
	GattClientService();
	GattClientService(GattClient& client, const BluetoothUuid& uuid, uint16_t start_handle, uint16_t end_handle);
	~GattClientService();

	GattClient& get_client() { return *this->client; }
	BluetoothUuid get_uuid() const { return this->uuid; }
	uint16_t get_start_handle() const { return this->start_handle; }
	uint16_t get_end_handle() const { return this->end_handle; }

	GattClientCharacteristic get_characteristic(const BluetoothUuid& uuid);

	operator bool() const { return this->client != nullptr; }
};


#endif //GATTCLIENTSERVICE_HPP__
