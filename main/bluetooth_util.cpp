#include "bluetooth_util.hpp"

// Bluetooth Base UUID 00000000-0000-1000-8000-00805F9B34FB
const uint8_t BluetoothUuid::base_uuid[16] = { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


const char GattClient::TAG[] = "GattClient";
std::aligned_storage<sizeof(GattClient::SharedContext), alignof(GattClient::SharedContext)>::type GattClient::shared_context_storage;
GattClient::SharedContext* GattClient::shared_context;

void GattClient::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
	freertos::LockGuard<freertos::Mutex> guard(shared_context->clients_mutex);
	for (auto& client : shared_context->clients) {
		if (event == ESP_GATTC_REG_EVT) {
			ESP_LOGI(TAG, "App registered. ID=%x", param->reg.app_id);
			if (param->reg.app_id == client->gattc_app_id) {
				client->gattc_if = gattc_if;
			}
		}
		client->handle_gattc_event_inner(event, gattc_if, param);
	}
}

std::shared_ptr<GattClientService> GattClient::get_service(const BluetoothUuid& uuid)
{
	if (!this->is_service_discovered) return nullptr;

	const auto& it = this->services.find(uuid);
	if (it == this->services.end()) return nullptr;

	
	auto shared = std::make_shared<GattClientService>(*this, it->second);
	this->register_eventsink(shared);
	return shared;
}

void GattClientService::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
	uint16_t conn_id = this->client.get_connection_id();

	switch (event)
	{
	case ESP_GATTC_GET_CHAR_EVT: {
		if (param->get_char.conn_id == conn_id && param->get_char.srvc_id.id.inst_id == this->id.id.inst_id) {
			if(param->get_char.status == ESP_GATT_OK) {
				const auto& char_id = param->get_char.char_id;
				auto it = this->characteristics.find(BluetoothUuid(char_id.uuid));
				if (it != this->characteristics.end()) {
					it->second->update_id(char_id);
				}
				else {
					auto shared = std::make_shared<GattClientCharacteristic>(*this, char_id);
					this->get_client().register_eventsink(shared);
					this->characteristics.insert(std::make_pair(BluetoothUuid(param->get_char.char_id.uuid), shared));
				}
				esp_ble_gattc_get_characteristic(gattc_if, conn_id, &param->get_char.srvc_id, &param->get_char.char_id);
			}
			else {
				if (this->enumerate_promise.is_valid()) {
					this->enumerate_promise.set_value(ESP_OK);
				}
			}
		}
		break;
	}
	default: break;
	}
}

freertos::future<esp_err_t> GattClientService::enumerate_characteristics()
{
	this->enumerate_promise.reset();

	esp_err_t result = esp_ble_gattc_get_characteristic(this->client.get_gattc_if(), this->client.get_connection_id(), &this->id, nullptr);
	if (result != ESP_OK) {
		this->enumerate_promise.set_value(result);
	}
	return this->enumerate_promise.get_future();
}

std::shared_ptr<GattClientCharacteristic> GattClientService::get_characteristic(const BluetoothUuid uuid) const
{
	const auto& it = this->characteristics.find(uuid);
	return it == this->characteristics.end() ? nullptr : it->second;
}


const char GattClientCharacteristic::TAG[] = "GattClientChar";
esp_err_t GattClientCharacteristic::begin_read_value()
{
	esp_gatt_srvc_id_t service_id = this->service.get_id();
	return esp_ble_gattc_read_char(this->get_gattc_if(), this->get_connection_id(), &service_id, &this->id, ESP_GATT_AUTH_REQ_NONE);
}

freertos::future<esp_err_t> GattClientCharacteristic::write_value_async(const uint8_t* buffer, size_t length)
{
	auto copied_data = new uint8_t[length];
	memcpy(copied_data, buffer, length);
	return this->write_value_async(std::shared_ptr<uint8_t>(copied_data), length);
}
freertos::future<esp_err_t> GattClientCharacteristic::write_value_async(const std::shared_ptr<uint8_t>& buffer, size_t length)
{
	this->value_write_context.data = buffer;
	this->value_write_context.length = length;
	auto service_id = this->service.get_id();
	this->value_write_promise.reset();
	esp_err_t result = esp_ble_gattc_write_char(this->get_gattc_if(), this->get_connection_id(), &service_id, &this->id, length, this->value_write_context.data.get(), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
	if (result != ESP_OK) {
		this->value_write_promise.set_value(result);
	}
	return this->value_write_promise.get_future();
}

freertos::future<esp_err_t> GattClientCharacteristic::enable_notification_async(bool enable)
{
	auto gattc_if = this->get_gattc_if();
	BdAddr bda = this->service.get_client().get_bd_addr();
	esp_gatt_srvc_id_t service_id = this->service.get_id();
	esp_gatt_status_t result;
	this->enable_notification_promise.reset();
	if (enable) {
		result = (esp_gatt_status_t)esp_ble_gattc_register_for_notify(gattc_if, bda.value, &service_id, &this->id);
	}
	else {
		result = (esp_gatt_status_t)esp_ble_gattc_unregister_for_notify(gattc_if, bda.value, &service_id, &this->id);
	}
	if (result != ESP_GATT_OK) {
		this->enable_notification_promise.set_value(result);
	}
	return this->enable_notification_promise.get_future();
}

GattClientCharacteristic::GattClientCharacteristic(GattClientService& service, esp_gatt_id_t id) : service(service), id(id), is_descriptor_enumeratred(false) {}

freertos::future<esp_err_t> GattClientCharacteristic::enumerate_descriptors()
{
	esp_gatt_srvc_id_t service_id = this->service.get_id();
	this->enumerate_descriptors_promise.reset();
	esp_err_t result = esp_ble_gattc_get_descriptor(this->get_gattc_if(), this->get_connection_id(), &service_id, &this->id, nullptr);
	if (result != ESP_OK) {
		this->enumerate_descriptors_promise.set_value(result);
	}
	return this->enumerate_descriptors_promise.get_future();
}

std::shared_ptr<GattClientDescriptor> GattClientCharacteristic::get_descriptor(const BluetoothUuid uuid) const
{
	const auto& it = this->descriptors.find(uuid);
	return it == this->descriptors.end() ? nullptr : it->second;
}

void GattClientCharacteristic::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
	auto connection_id = this->service.get_client().get_connection_id();
	auto service_inst_id = this->service.get_id().id.inst_id;
	auto char_inst_id = this->id.inst_id;

	switch (event)
	{
	case ESP_GATTC_READ_CHAR_EVT: {

		if (param->read.conn_id == connection_id && param->read.char_id.inst_id == id.inst_id) {
			if (this->value_read_handler) {
				this->value_read_handler(param->read.status, param->read.value, param->read.value_len);
			}
		}
		break;
	}
	case ESP_GATTC_WRITE_CHAR_EVT: {
		if (param->write.conn_id == connection_id && param->write.char_id.inst_id == id.inst_id) {
			if (this->value_write_promise.is_valid()) {
				this->value_write_promise.set_value(param->write.status);
			}
		}
		break;
	}
	case ESP_GATTC_GET_DESCR_EVT: {
		if (param->get_descr.conn_id == connection_id && param->get_descr.char_id.inst_id == this->id.inst_id) {
			if (param->get_descr.status == ESP_GATT_OK) {
				esp_gatt_id_t descr_id = param->get_descr.descr_id;
				auto it = this->descriptors.find(BluetoothUuid(descr_id.uuid));
				if (it == this->descriptors.end()) {
					auto shared = std::make_shared<GattClientDescriptor>(*this, descr_id);
					this->service.get_client().register_eventsink(shared);
					this->descriptors.insert(std::make_pair(BluetoothUuid(descr_id.uuid), shared));
				}
				BluetoothUuid uuid(param->get_descr.descr_id.uuid);
				char uuid_str[100];
				uuid.to_string(uuid_str);
				ESP_LOGI(TAG, "DESCR: %s", uuid_str);
				esp_ble_gattc_get_descriptor(gattc_if, connection_id, &param->get_descr.srvc_id, &param->get_descr.char_id, &param->get_descr.descr_id);
			}
			else {
				if (this->enumerate_descriptors_promise.is_valid()) {
					this->enumerate_descriptors_promise.set_value(param->get_descr.status);
				}
				this->is_descriptor_enumeratred = true;
			}
		}
		break;
	}
	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		if (param->reg_for_notify.srvc_id.id.inst_id == service_inst_id && param->reg_for_notify.char_id.inst_id == char_inst_id) {
			if (this->enable_notification_promise.is_valid()) {
				this->enable_notification_promise.set_value(param->reg_for_notify.status);
			}
		}
		break;
	}
	case ESP_GATTC_UNREG_FOR_NOTIFY_EVT: {
		if (param->unreg_for_notify.srvc_id.id.inst_id == service_inst_id && param->unreg_for_notify.char_id.inst_id == char_inst_id) {
			if (this->enable_notification_promise.is_valid()) {
				this->enable_notification_promise.set_value(param->unreg_for_notify.status);
			}
		}
		break;
	}
	case ESP_GATTC_NOTIFY_EVT: {
		if (param->notify.conn_id == connection_id && param->notify.char_id.inst_id == this->id.inst_id) {
			if (this->notification_handler) {
				this->notification_handler(param->notify.value, param->notify.value_len);
			}
		}
		break;
	}
	default: break;
	}
}

GattClientDescriptor::GattClientDescriptor(GattClientCharacteristic & characteristic, esp_gatt_id_t id) : characteristic(characteristic), id(id) {}

freertos::future<esp_err_t> GattClientDescriptor::write_value_async(const uint8_t * buffer, size_t length)
{
	auto copied_data = new uint8_t[length];
	memcpy(copied_data, buffer, length);
	return this->write_value_async(std::shared_ptr<uint8_t>(copied_data), length);
}

freertos::future<esp_err_t> GattClientDescriptor::write_value_async(const std::shared_ptr<uint8_t>& buffer, size_t length)
{
	this->write_context.data = buffer;
	this->write_context.length = length;
	auto service_id = this->characteristic.service.get_id();
	auto char_id = this->characteristic.get_id();
	this->write_promise.reset();
	esp_err_t result = esp_ble_gattc_write_char_descr(this->get_gattc_if(), this->get_connection_id(), &service_id, &char_id, &this->id, length, this->write_context.data.get(), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
	if (result != ESP_OK) {
		this->write_promise.set_value(result);
	}
	return this->write_promise.get_future();
}

void GattClientDescriptor::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param)
{
	auto connection_id = this->characteristic.get_connection_id();
	switch (event)
	{
	case ESP_GATTC_WRITE_DESCR_EVT: {
		if (param->write.conn_id == connection_id && param->write.char_id.inst_id == id.inst_id) {
			if (this->write_promise.is_valid()) {
				this->write_promise.set_value(param->write.status);
			}
		}
		break;
	}
	default: break;
	}
}
