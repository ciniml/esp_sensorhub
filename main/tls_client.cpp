#include "tls_client.hpp"
#include <esp_log.h>
#include <functional>
#include <type_traits>

const char* TlsClient::TAG = "TlsClient";

template<typename TFunction>
struct Cleaner
{
	bool is_cancelled;
	TFunction cleaner;
	Cleaner(TFunction&& cleaner) : is_cancelled(false), cleaner(cleaner) {}
	Cleaner(Cleaner&& cleaner) : is_cancelled(cleaner.is_cancelled), cleaner(std::move(cleaner.cleaner)) {}
	void cancel() { this->is_cancelled = true; }
	~Cleaner()
	{
		if (!this->is_cancelled) {
			this->cleaner();
		}
	}
};
template<typename TFunction>
static Cleaner<TFunction> make_cleaner(TFunction&& cleaner) { return Cleaner<TFunction>(std::forward<TFunction>(cleaner)); }

bool TlsClient::initialize(const  std::uint8_t* cert_pem, std::size_t cert_pem_length)
{
	int ret = 0;

	ESP_LOGI(TAG, "Initializing mbedtls");

	mbedtls_ssl_init(&this->ssl);
	mbedtls_x509_crt_init(&this->cacert);
	mbedtls_ctr_drbg_init(&this->ctr_drbg);

	mbedtls_ssl_config_init(&this->ssl_config);
	mbedtls_entropy_init(&this->entropy);

	if ((ret = mbedtls_ctr_drbg_seed(&this->ctr_drbg, &mbedtls_entropy_func, &this->entropy, nullptr, 0)) != 0) {
		ESP_LOGE(TAG, "mbedtls_ctr_drbg_seed failed. ret=0x%x", ret);
		return false;
	}

	//ret = mbedtls_x509_crt_parse(&this->cacert, cert_pem, cert_pem_length);
	//if (ret < 0) {
	//	ESP_LOGE(TAG, "mbedtls_x509_crt_parse failed. ret=0x%x", ret);
	//	return false;
	//}

	if ((ret = mbedtls_ssl_config_defaults(&this->ssl_config, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
		ESP_LOGE(TAG, "mbedtls_ssl_config_defaults failed. 0x%x", ret);
		return false;
	}

	//mbedtls_ssl_conf_authmode(&this->ssl_config, MBEDTLS_SSL_VERIFY_REQUIRED);
	mbedtls_ssl_conf_authmode(&this->ssl_config, MBEDTLS_SSL_VERIFY_OPTIONAL);
	mbedtls_ssl_conf_ca_chain(&this->ssl_config, &this->cacert, nullptr);
	mbedtls_ssl_conf_rng(&this->ssl_config, mbedtls_ctr_drbg_random, &this->ctr_drbg);

#ifdef CONFIG_MBEDTLS_DEBUG
	mbedtls_esp_enable_debug_log(&this->ssl_config, 4);
#endif

	if ((ret = mbedtls_ssl_setup(&this->ssl, &this->ssl_config)) != 0) {
		ESP_LOGE(TAG, "mbedtls_ssl_setup failed 0x%x", ret);
		return false;
	}

	this->state = State::NotConnected;

	return true;
}

bool TlsClient::connect(const char* host, const char* port)
{
	int ret = 0;

	if (host == nullptr || port == nullptr) {
		return false;
	}

	if (this->state != State::NotConnected) {
		return false;
	}

	ret = mbedtls_net_connect(&this->server_fd, host, port, MBEDTLS_NET_PROTO_TCP);
	if (ret != 0) {
		ESP_LOGE(TAG, "mbedtls_net_connect failed. ret=0x%x", ret);
		return false;
	}

	// Cleanup sesion and FD on error.
	auto server_fd_cleaner = make_cleaner([this]() {
		ESP_LOGI(TAG, "cleanup connection resources.");
		mbedtls_ssl_session_reset(&this->ssl);
		mbedtls_net_free(&this->server_fd); 
	});

	ESP_LOGI(TAG, "connected to %s:%s", host, port);

	mbedtls_ssl_set_bio(&this->ssl, &this->server_fd, mbedtls_net_send, mbedtls_net_recv, nullptr);

	while ((ret = mbedtls_ssl_handshake(&this->ssl)) != 0) {
		if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			ESP_LOGE(TAG, "mbedtls_ssl_set_bio failed. ret=0x%x", ret);
			return false;
		}
	}

	int flags = mbedtls_ssl_get_verify_result(&this->ssl);
	if (flags != 0) {
		ESP_LOGW(TAG, "TLS verification failed. flags=0x%x", flags);
		//return false;
	}

	ESP_LOGI(TAG, "handshake completed.");

	this->state = State::Connected;
	server_fd_cleaner.cancel();	// Cancel to cleanup sesson and file descriptor.
	return true;
}

void TlsClient::disconnect()
{
	if (this->state != State::Connected) return;

	mbedtls_ssl_close_notify(&this->ssl);
	mbedtls_ssl_session_reset(&this->ssl);
	mbedtls_net_free(&this->server_fd);

	this->state = State::NotConnected;
}


int TlsClient::read(std::uint8_t* data, std::size_t length)
{
	if (data == nullptr || length == 0 || this->state != State::Connected) {
		return -1;
	}

	int ret = 0;
	while ((ret = mbedtls_ssl_read(&this->ssl, data, length)) < 0) {
		if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			ESP_LOGE(TAG, "read failed. ret=0x%x", ret);
			return ret;
		}
	}

	return ret;
}

int TlsClient::write(const std::uint8_t * data, std::size_t length)
{
	if (data == nullptr || length == 0 || this->state != State::Connected) {
		return -1;
	}

	int ret = 0;
	while ((ret = mbedtls_ssl_write(&this->ssl, data, length)) < 0) {
		if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			ESP_LOGE(TAG, "read failed. ret=0x%x", ret);
			return ret;
		}
	}

	return ret;
}

void TlsClient::terminate()
{
}
