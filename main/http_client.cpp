#include "http_client.hpp"
#include <esp_log.h>
#include <functional>
#include <type_traits>
#include <memory>
#include <cstring>

const char* HttpClient::TAG = "HttpClient";


bool HttpClient::write(const std::uint8_t * data, size_t length)
{
	if (this->client.is_connected()) return false;

	size_t remaining = length;
	const std::uint8_t* p = data;

	while (remaining > 0) {
		int result = this->client.write(p, remaining);
		if (result < 0) return false;

		remaining -= result;
		p += result;
	}
	return true;
}

HttpClient::HttpClient(ClientType & client) : client(client), user_agent("ESP32 HTTP Client")
{
	this->parser_context.client = this;
}

int HttpClient::response_parser_on_header_field(http_parser* parser, const char *at, size_t length)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	self->parser_context.header_name.clear();
	self->parser_context.header_name.append(at, length);
	return 0;
}
int HttpClient::response_parser_on_header_value(http_parser* parser, const char *at, size_t length)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	self->parser_context.header_value.clear();
	self->parser_context.header_value.append(at, length);
	if (self->parser_context.receiver != nullptr) {
		return self->parser_context.receiver->on_header(*self, self->parser_context.header_name, self->parser_context.header_value);
	}
	return 0;
}

int HttpClient::response_parser_on_body(http_parser* parser, const char *at, size_t length)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	if (self->parser_context.receiver != nullptr) {
		return self->parser_context.receiver->on_body(*self, reinterpret_cast<const std::uint8_t*>(at), length);
	}
	return 0;
}

int HttpClient::response_parser_on_message_begin(http_parser* parser)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	if (self->parser_context.receiver != nullptr) {
		return self->parser_context.receiver->on_message_begin(*self);
	}
	return 0;
}

int HttpClient::response_parser_on_message_complete(http_parser* parser)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	if (self->parser_context.receiver != nullptr) {
		return self->parser_context.receiver->on_message_complete(*self);
	}
	return 0;
}

int HttpClient::response_parser_on_header_complete(http_parser* parser)
{
	HttpClient* self = reinterpret_cast<HttpClient*>(parser->data);
	if (self->parser_context.receiver != nullptr) {
		HttpResponseInfo info;
		info.http_major		= parser->http_major ;
		info.http_minor		= parser->http_minor ;
		info.http_errno		= parser->http_errno ;
		info.status_code	= parser->status_code;
		info.content_length = parser->content_length;

		return self->parser_context.receiver->on_header_complete(*self, info);
	}
	return 0;
}

void HttpClient::initialize_parser_settings(http_parser_settings& settings)
{
	http_parser_settings_init(&settings);
	settings.on_message_begin = &response_parser_on_message_begin;
	settings.on_message_complete = &response_parser_on_message_complete;
	settings.on_headers_complete = &response_parser_on_header_complete;
	settings.on_header_field = &response_parser_on_header_field;
	settings.on_header_value = &response_parser_on_header_value;
	settings.on_body = &response_parser_on_body;
}


bool HttpClient::get(const char * host, const char * port, const char * request, IHttpResponseReceiver& receiver)
{
	if (this->client.is_connected()) return false;

	ESP_LOGI(TAG, "GET: %s:%s%s", host, port, request);

	{
		std::unique_ptr<char> buffer(new char[2048]);
		char* p = buffer.get();

		if (!this->client.connect(host, port)) {
			ESP_LOGE(TAG, "connection failed.");
			return false;
		}

		p += sprintf(p, "GET %s HTTP/1.1\r\n", request);
		p += sprintf(p, "Host: %s\r\n", host);
		p += sprintf(p, "User-Agent: %s\r\n", this->user_agent.c_str());
		p += sprintf(p, "Connection: close\r\n");
		p += sprintf(p, "Accept-Encoding: identity;q=1,chunked;q=0.1,*;q=0\r\n");
		strcat(p, "\r\n"); p += 2;

		char* head = buffer.get();
		this->client.write(reinterpret_cast<const std::uint8_t*>(head), p - head);
	}

	{
		constexpr size_t BUFFER_SIZE = 2048;
		std::unique_ptr<std::uint8_t> buffer(new std::uint8_t[BUFFER_SIZE]);
		if (!buffer) {
			ESP_LOGE(TAG, "failed to allocate read buffer.");
			return false;
		}

		// store handlers
		this->parser_context.receiver = &receiver;

		// Initialize response parser
		http_parser_init(&this->response_parser, http_parser_type::HTTP_RESPONSE);
		this->response_parser.data = this;
		http_parser_settings settings;
		this->initialize_parser_settings(settings);
		
		// Receive responses and parse them.
		size_t bytes_remaining = 0;
		while(true)
		{
			int ret = this->client.read(buffer.get() + bytes_remaining, BUFFER_SIZE - bytes_remaining);
			ESP_LOGI(TAG, "client.read returned %d.", ret);
			if (ret < 0) {
				ESP_LOGE(TAG, "failed to read.");
				this->client.disconnect();
				return false;
			}
			
			bytes_remaining += ret;
			size_t bytes_processed = http_parser_execute(&this->response_parser, &settings, reinterpret_cast<char*>(buffer.get()), bytes_remaining);
			
			bytes_remaining -= bytes_processed;
			if (bytes_remaining == 0 || (ret == 0 && bytes_processed == 0)) {
				break;
			}
			if (bytes_remaining > 0) {
				auto buffer_body = buffer.get();
				std::memcpy(buffer_body, buffer_body + bytes_processed, bytes_remaining);
			}
		}
	}
	
	this->response_parser.data = nullptr;

	this->client.disconnect();

	ESP_LOGI(TAG, "GET: %s:%s%s completed", host, port, request);

	return true;
}
