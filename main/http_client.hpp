#ifndef HTTP_CLIENT_HPP__
#define HTTP_CLIENT_HPP__

#include <sstream>
#include <string>
#include <map>

#include <functional>

#include <esp_http_client.h>

struct HttpResponseInfo
{
	std::uint16_t status_code;
	std::size_t content_length;
};

class HttpClient;

struct IHttpResponseReceiver
{
	virtual int on_header(const HttpClient& client, const std::string& name, const std::string& value) { return 0; }
	virtual int on_body(const HttpClient& client, const std::uint8_t* buffer, std::size_t length) { return 0; }
};

class HttpClientHandlerInvoker;

class HttpClient
{
private:
	esp_http_client_config_t config;
	const char* cert_pem;
	IHttpResponseReceiver* receiver;

	esp_err_t handle_event(esp_http_client_event_t* evt);
	bool receive_response(IHttpResponseReceiver& receiver);
public:
	HttpClient();
	HttpClient(const char* cert_pem);

	bool perform(esp_http_client_method_t method, const char* url, const char* content_type, const void* content, std::size_t content_length, IHttpResponseReceiver& receiver, HttpResponseInfo& response);

	friend HttpClientHandlerInvoker;
};

#endif //HTTP_CLIENT_HPP__
