#include "bluetooth_util.hpp"
#include "gattclient.hpp"
#include "gattclientservice.hpp"
#include "gattclientcharacteristic.hpp"

// Bluetooth Base UUID 00000000-0000-1000-8000-00805F9B34FB
const uint8_t BluetoothUuid::base_uuid[16] = { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


const char GattClient::TAG[] = "GattClient";
std::aligned_storage<sizeof(GattClient::SharedContext), alignof(GattClient::SharedContext)>::type GattClient::shared_context_storage;
GattClient::SharedContext* GattClient::shared_context;

void GattClientService::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param)
{
}

GattClientService::GattClientService() : client(nullptr) {}
GattClientService::GattClientService(GattClient& client, const BluetoothUuid& uuid, uint16_t start_handle, uint16_t end_handle) : client(&client), uuid(uuid), start_handle(start_handle), end_handle(end_handle)
{
	client.register_eventsink(this);
}
GattClientService::~GattClientService()
{
	if (this->client != nullptr) {
		this->client->unregister_eventsink(this);
	}
	this->client = nullptr;
}

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

void GattClient::handle_gattc_event_inner(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param)
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
			this->state = State::Connected;
			ESP_LOGI(TAG, "Device connected.");
			esp_err_t result = esp_ble_gattc_search_service(this->gattc_if, this->conn_id, nullptr);
			if (result != ESP_OK) {
				ESP_LOGE(TAG, "esp_ble_gattc_search_service returned %d\n", result);
			}
		}
		break;
	}
	case ESP_GATTC_CFG_MTU_EVT: {
		break;
	}
	case ESP_GATTC_SRVC_CHG_EVT: {
		if (this->bd_addr == param->srvc_chg.remote_bda) {
			esp_ble_gattc_cache_refresh(param->srvc_chg.remote_bda);
			ESP_LOGI(TAG, "Service changed.");
		}
		break;
	}
	case ESP_GATTC_SEARCH_RES_EVT: {    // Service discovery result.
		if (param->search_res.conn_id == this->conn_id) {
			BluetoothUuid uuid(param->search_res.srvc_id.uuid);
			char buffer[128];
			uuid.to_string(buffer);
			ESP_LOGI(TAG, "Service discovered. UUID=%s", buffer);
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
		ptr->handle_gattc_event(event, gattc_if, param);
	}
}


GattClientService GattClient::get_service(const BluetoothUuid& uuid, std::size_t index)
{
	esp_gattc_service_elem_t element;
	uint16_t count = 1;
	auto esp_uuid = uuid.value;

	esp_err_t result = esp_ble_gattc_get_service(this->gattc_if, this->conn_id, &esp_uuid, &element, &count, index);
	if (result != ESP_OK) {
		return GattClientService();
	}

	return GattClientService(*this, uuid, element.start_handle, element.end_handle);
}


GattClientCharacteristic GattClientService::get_characteristic(const BluetoothUuid& uuid)
{
	esp_gattc_char_elem_t element;
	uint16_t count = 1;
	auto esp_uuid = uuid.value;

	esp_err_t result = esp_ble_gattc_get_char_by_uuid(this->client->get_gattc_if() , this->client->get_connection_id(), this->start_handle, this->end_handle, uuid.value, &element, &count);
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid returned %d", result);
		return GattClientCharacteristic();
	}

	return std::move(GattClientCharacteristic(*this->client, element));
}

const char GattClientService::TAG[] = "GattClientService";
const char GattClientCharacteristic::TAG[] = "GattClientChar";
esp_err_t GattClientCharacteristic::begin_read_value()
{
	return esp_ble_gattc_read_char(this->get_gattc_if(), this->get_connection_id(), this->element.char_handle, ESP_GATT_AUTH_REQ_NONE);
}

freertos::future<esp_err_t> GattClientCharacteristic::write_value_async(const uint8_t* buffer, std::size_t length)
{
	this->value_write_promise.reset();
	esp_err_t result = esp_ble_gattc_write_char(this->get_gattc_if(), this->get_connection_id(), this->element.char_handle, length, const_cast<uint8_t*>(buffer), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "esp_ble_gattc_write_char returned %d", result);
		this->value_write_promise.set_value(result);
	}
	return this->value_write_promise.get_future();
}

freertos::future<esp_err_t> GattClientCharacteristic::enable_notification_async(bool enable)
{
	auto gattc_if = this->get_gattc_if();
	BdAddr bda = this->client->get_bd_addr();
	esp_gatt_status_t result;
	this->enable_notification_promise.reset();
	if (enable) {
		result = (esp_gatt_status_t)esp_ble_gattc_register_for_notify(gattc_if, bda.value, this->element.char_handle);
	}
	else {
		result = (esp_gatt_status_t)esp_ble_gattc_unregister_for_notify(gattc_if, bda.value, this->element.char_handle);
	}
	if (result != ESP_GATT_OK) {
		this->enable_notification_promise.set_value(result);
	}
	return this->enable_notification_promise.get_future();
}

uint16_t GattClientCharacteristic::get_connection_id() const
{
	return this->client->get_connection_id();
}

esp_gatt_if_t GattClientCharacteristic::get_gattc_if() const
{
	return this->client->get_gattc_if();
}

GattClientCharacteristic::GattClientCharacteristic(GattClient& client, const esp_gattc_char_elem_t& element) : client(&client), element(element) {}

GattClientCharacteristic::GattClientCharacteristic(GattClientCharacteristic&& obj)
{
	if (obj.client != nullptr) {
		this->client = obj.client;
		this->element = obj.element;
		obj.client->register_eventsink(this);
	}
}
GattClientCharacteristic::~GattClientCharacteristic()
{
	if (this->client != nullptr) {
		this->client->unregister_eventsink(this);
	}
}
GattClientCharacteristic& GattClientCharacteristic::operator=(const GattClientCharacteristic& rhs) 
{
	if (this->client == nullptr) {
		this->client = rhs.client;
		this->element = rhs.element;
		this->client->register_eventsink(this);
	}
	return *this;
}

GattClientDescriptor GattClientCharacteristic::get_descriptor(const BluetoothUuid uuid)
{
	esp_gattc_descr_elem_t element;
	uint16_t count = 1;
	auto esp_uuid = uuid.value;

	esp_err_t result = esp_ble_gattc_get_descr_by_char_handle(this->get_gattc_if(), this->get_connection_id(), this->element.char_handle, uuid.value, &element, &count);
	if (result != ESP_OK || count == 0) {
		ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle returned %d", result);
		return GattClientDescriptor();
	}
	
	return std::move(GattClientDescriptor(*this->client, element));
}


void GattClientCharacteristic::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
	auto connection_id = this->get_connection_id();
	
	switch (event)
	{
	case ESP_GATTC_READ_CHAR_EVT: {

		if (param->read.conn_id == connection_id && param->read.handle == this->element.char_handle) {
			if (this->value_read_handler) {
				this->value_read_handler(param->read.status, param->read.value, param->read.value_len);
			}
		}
		break;
	}
	case ESP_GATTC_WRITE_CHAR_EVT: {
		ESP_LOGI(TAG, "WRITE_CHAR\n");
		if (param->write.conn_id == connection_id && param->write.handle == this->element.char_handle) {
			if (this->value_write_promise.is_valid()) {
				this->value_write_promise.set_value(param->write.status);
			}
		}
		break;
	}
	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		if (param->reg_for_notify.handle == this->element.char_handle) {
			if (this->enable_notification_promise.is_valid()) {
				this->enable_notification_promise.set_value(param->reg_for_notify.status);
			}
		}
		break;
	}
	case ESP_GATTC_UNREG_FOR_NOTIFY_EVT: {
		if (param->unreg_for_notify.handle == this->element.char_handle ) {
			if (this->enable_notification_promise.is_valid()) {
				this->enable_notification_promise.set_value(param->unreg_for_notify.status);
			}
		}
		break;
	}
	case ESP_GATTC_NOTIFY_EVT: {
		if (param->notify.conn_id == connection_id && param->notify.handle == this->element.char_handle) {
			if (this->notification_handler) {
				this->notification_handler(param->notify.value, param->notify.value_len);
			}
		}
		break;
	}
	default: break;
	}
}

uint16_t GattClientDescriptor::get_connection_id() const
{
	return this->client->get_connection_id();
}

esp_gatt_if_t GattClientDescriptor::get_gattc_if() const
{
	return this->client->get_gattc_if();
}

GattClientDescriptor::GattClientDescriptor(GattClient & client, const esp_gattc_descr_elem_t & element) : client(&client), element(element) {}

GattClientDescriptor::GattClientDescriptor(GattClientDescriptor&& obj)
{
	if (obj.client != nullptr) {
		this->client = obj.client;
		this->element = obj.element;
		this->client->register_eventsink(this);
	}
}
GattClientDescriptor::~GattClientDescriptor()
{
	if (this->client != nullptr) {
		this->client->unregister_eventsink(this);
	}
}
GattClientDescriptor& GattClientDescriptor::operator=(const GattClientDescriptor& rhs)
{
	if (this->client == nullptr) {
		this->client = rhs.client;
		this->element = rhs.element;
		this->client->register_eventsink(this);
	}
	return *this;
}

freertos::future<esp_err_t> GattClientDescriptor::write_value_async(const uint8_t * buffer, std::size_t length)
{
	this->write_promise.reset();
	esp_err_t result = esp_ble_gattc_write_char_descr(this->get_gattc_if(), this->get_connection_id(), this->element.handle, length, const_cast<uint8_t*>(buffer), ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
	if (result != ESP_OK) {
		this->write_promise.set_value(result);
	}
	return this->write_promise.get_future();
}

void GattClientDescriptor::handle_gattc_event(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param)
{
	auto connection_id = this->get_connection_id();
	switch (event)
	{
	case ESP_GATTC_WRITE_DESCR_EVT: {
		if (param->write.conn_id == connection_id && param->write.handle == this->element.handle) {
			if (this->write_promise.is_valid()) {
				this->write_promise.set_value(param->write.status);
			}
		}
		break;
	}
	default: break;
	}
}
