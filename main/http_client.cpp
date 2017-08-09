#include "http_client.hpp"
#include <esp_log.h>
#include <functional>
#include <type_traits>
#include <memory>

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
	// Initialize response parser
	http_parser_init(&this->response_parser, http_parser_type::HTTP_RESPONSE);
}

bool HttpClient::get(const char * host, const char * port, const char * request, std::stringstream& response)
{
	if (this->client.is_connected()) return false;
	
	ESP_LOGI(TAG, "GET: %s:%s%s", host, port, request);

	{
		std::stringstream buffer;

		if (!this->client.connect(host, port)) {
			ESP_LOGE(TAG, "connection failed.");
			return false;
		}

		buffer << "GET " << request << " HTTP/1.1" << std::endl;
		buffer << "Host: " << host << std::endl;
		buffer << "User-Agent: " << this->user_agent << std::endl;
		buffer << "Connection: close" << std::endl;
		buffer << "Accept-Encoding: identity;q=1,chunked;q=0.1,*;q=0" << std::endl;
		buffer << std::endl;

		auto header = buffer.str();
		this->client.write(reinterpret_cast<const std::uint8_t*>(header.c_str()), header.size());
	}

	{
		std::unique_ptr<std::uint8_t> buffer(new std::uint8_t[2048]);
		if (!buffer) {
			ESP_LOGE(TAG, "failed to allocate read buffer.");
			return false;
		}

		bool is_firstline_detected = false;
		while(true)
		{
			int ret = this->client.read(buffer.get(), 2048);
			if (ret < 0) {
				ESP_LOGE(TAG, "failed to read.");
				return false;
			}
			if (!is_firstline_detected) {
				const uint8_t* p = buffer.get();
				for (size_t i = 0; i < ret; i++) {
					if (p[i] == '\n') {
						is_firstline_detected = true;
						break;
					}
				}
			}
			else if (ret == 0) {
				break;
			}

			response.write(reinterpret_cast<const char*>(buffer.get()), ret);
		}
	}
	
	this->client.disconnect();

	ESP_LOGI(TAG, "GET: %s:%s%s completed", host, port, request);

	return true;
}
