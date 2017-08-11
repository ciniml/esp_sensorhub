#ifndef HTTP_CLIENT_HPP__
#define HTTP_CLIENT_HPP__

#include "tls_client.hpp"
#include <sstream>
#include <string>
#include <map>

#include <http_parser.h>
#include <functional>


struct HttpResponseInfo
{
	std::uint16_t http_major;
	std::uint16_t http_minor;
	std::uint16_t status_code;
	std::uint8_t http_errno;
	std::size_t content_length;
};

class HttpClient;

struct IHttpResponseReceiver
{
	virtual int on_message_begin(const HttpClient& client) { return 0; }
	virtual int on_message_complete(const HttpClient& client) { return 0; }
	virtual int on_header_complete(const HttpClient& client, const HttpResponseInfo& info) { return 0; }
	virtual int on_header(const HttpClient& client, const std::string& name, const std::string& value) { return 0; }
	virtual int on_body(const HttpClient& client, const std::uint8_t* buffer, std::size_t length) { return 0; }
};

class HttpClient
{
public:
	typedef TlsClient ClientType;
private:
	
	static int response_parser_on_header_field(http_parser * parser, const char * at, size_t length);
	static int response_parser_on_header_value(http_parser * parser, const char * at, size_t length);
	static int response_parser_on_body(http_parser * parser, const char * at, size_t length);
	static int response_parser_on_message_begin(http_parser * parser);
	static int response_parser_on_message_complete(http_parser * parser);
	static int response_parser_on_header_complete(http_parser * parser);
	

	static const char* TAG;
	ClientType& client;
	std::string user_agent;
	struct HttpParserContext
	{
		HttpClient* client;
		std::string header_name;
		std::string header_value;
		IHttpResponseReceiver* receiver;
	} parser_context;
	
	http_parser response_parser;

	void initialize_parser_settings(http_parser_settings& settings);

	bool write(const std::uint8_t* data, size_t length);
public:
	HttpClient(ClientType& client);

	bool get(const char* host, const char* port, const char* request, IHttpResponseReceiver& receiver);
};

#endif //HTTP_CLIENT_HPP__
