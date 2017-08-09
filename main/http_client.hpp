#ifndef HTTP_CLIENT_HPP__
#define HTTP_CLIENT_HPP__

#include "tls_client.hpp"
#include <sstream>
#include <string>

#include <http_parser.h>

class HttpClient
{
public:
	typedef TlsClient ClientType;

private:
	static const char* TAG;
	ClientType& client;
	std::string user_agent;
	http_parser response_parser;

	bool write(const std::uint8_t* data, size_t length);
public:
	HttpClient(ClientType& client);

	bool get(const char* host, const char* port, const char* request, std::stringstream& response);
};

#endif //HTTP_CLIENT_HPP__
