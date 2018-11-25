#include "http_client.hpp"
#include <esp_log.h>
#include <functional>
#include <type_traits>
#include <memory>
#include <cstring>

static const char* TAG = "HttpClient";

HttpClient::HttpClient() : cert_pem(nullptr) {}
HttpClient::HttpClient(const char* cert_pem) : cert_pem(cert_pem) {}

class HttpClientHandlerInvoker
{
public:
	static esp_err_t http_event_handler(esp_http_client_event_t* evt)
	{
		auto self = static_cast<HttpClient*>(evt->user_data);
		if( self != nullptr ) {
			return self->handle_event(evt);
		}
		else {
			return ESP_OK;
		}
	}
};

esp_err_t HttpClient::handle_event(esp_http_client_event_t* evt)
{
	switch(evt->event_id) {
	case HTTP_EVENT_ERROR:
		ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
		break;
	case HTTP_EVENT_ON_CONNECTED:
		ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
		break;
	case HTTP_EVENT_HEADER_SENT:
		ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
		break;
	case HTTP_EVENT_ON_HEADER:
	{
		if( evt->header_key != nullptr ) {
			if( evt->header_value != nullptr ) {
				ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER: %s=%s", evt->header_key, evt->header_value);
			}
			else {
				ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER: %s", evt->header_key);
			}
			if( this->receiver != nullptr ) {
				this->receiver->on_header(*this, evt->header_key, evt->header_value);
			}
		}
		break;
	}
	case HTTP_EVENT_ON_DATA:
	{
		auto data = reinterpret_cast<std::uint8_t*>(evt->data);
		if( data != nullptr ) {
			ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER %.*s", evt->data_len, data);
		}
		if( this->receiver != nullptr ) {
			this->receiver->on_body(*this, data, evt->data_len);
		}
		break;
	}
	case HTTP_EVENT_ON_FINISH:
		ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
		break;
	case HTTP_EVENT_DISCONNECTED:
		ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
		break;
	}
	return ESP_OK;
}

typedef std::unique_ptr<struct esp_http_client, decltype(&esp_http_client_cleanup)> HttpClientHandle;

bool HttpClient::perform(esp_http_client_method_t method, const char* url, const char* content_type, const void* content, std::size_t content_length, IHttpResponseReceiver& receiver, HttpResponseInfo& response)
{
	memset(&this->config, 0, sizeof(this->config));
	this->config.user_data = this;
	this->config.event_handler = HttpClientHandlerInvoker::http_event_handler;
	this->config.url = url;
	this->config.cert_pem = this->cert_pem;
	this->config.method = method;
	
	if( content_type != nullptr ) {
		content_type = "application/json";
	}

	HttpClientHandle client(esp_http_client_init(&this->config), &esp_http_client_cleanup);
	if( !client ) {
		ESP_LOGE(TAG, "Error: failed to initialize HTTP client");
		return false;
	}

	esp_err_t err = ESP_OK;
	if( content != nullptr ) {
		err = esp_http_client_set_header(client.get(), "Content-Type", content_type);
		if( err != ESP_OK ) {
			ESP_LOGE(TAG, "Error: failed to set content-type field err=%d", err);
			return false;
		}
		err = esp_http_client_set_post_field(client.get(), reinterpret_cast<const char*>(content), content_length);
		if( err != ESP_OK ) {
			ESP_LOGE(TAG, "Error: failed to set post field err=%d", err);
			return false;
		}
	}
	err = esp_http_client_perform(client.get());
	if( err != ESP_OK ) {
		ESP_LOGE(TAG, "Error: failed to perform HTTP request err=%d", err);
	}

	response.status_code    = esp_http_client_get_status_code(client.get());
	response.content_length = esp_http_client_get_content_length(client.get());
	return err == ESP_OK;
}
