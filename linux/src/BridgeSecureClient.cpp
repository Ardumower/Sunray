#ifdef __linux__

#include "BridgeSecureClient.h"

static void ensure_openssl_init() {
  static bool inited = false;
  if (!inited) {
    SSL_load_error_strings();
    OpenSSL_add_ssl_algorithms();
    inited = true;
  }
}

BridgeSecureClient::~BridgeSecureClient() {
  stop();
}

static void logSSLErrors(const char* where) {
  unsigned long e;
  while ((e = ERR_get_error()) != 0) {
    char buf[256];
    ERR_error_string_n(e, buf, sizeof(buf));
    fprintf(stderr, "TLS %s: %s\n", where, buf);
  }
}

bool BridgeSecureClient::tlsConnect(int sock, const char* host) {
  ensure_openssl_init();
  _ctx = SSL_CTX_new(TLS_client_method());
  if (!_ctx) { logSSLErrors("SSL_CTX_new"); return false; }

  // Load CA trust
  if (_caPath && *_caPath) {
    if (strcmp(_caPath, "SYSTEM") == 0) {
      if (!SSL_CTX_set_default_verify_paths(_ctx)) { logSSLErrors("set_default_verify_paths"); return false; }
    } else {
      if (!SSL_CTX_load_verify_locations(_ctx, _caPath, nullptr)) { logSSLErrors("load_verify_locations"); return false; }
    }
  } else {
    // No path provided: fall back to system default CA store
    if (!SSL_CTX_set_default_verify_paths(_ctx)) { logSSLErrors("set_default_verify_paths"); return false; }
  }

  // Load client cert/key (for mTLS)
  if (_certPath && *_certPath) {
    if (SSL_CTX_use_certificate_file(_ctx, _certPath, SSL_FILETYPE_PEM) <= 0) { logSSLErrors("use_certificate_file"); return false; }
  }
  if (_keyPath && *_keyPath) {
    if (SSL_CTX_use_PrivateKey_file(_ctx, _keyPath, SSL_FILETYPE_PEM) <= 0) { logSSLErrors("use_PrivateKey_file"); return false; }
    if (!SSL_CTX_check_private_key(_ctx)) { fprintf(stderr, "TLS: private key does not match certificate\n"); return false; }
  }

  _ssl = SSL_new(_ctx);
  if (!_ssl) { logSSLErrors("SSL_new"); return false; }
  SSL_set_fd(_ssl, sock);
  const char* sni = (_serverName && *_serverName) ? _serverName : host;
  if (sni && *sni) SSL_set_tlsext_host_name(_ssl, sni);
  SSL_set_verify(_ssl, SSL_VERIFY_PEER, nullptr);

  if (SSL_connect(_ssl) != 1) { logSSLErrors("SSL_connect"); return false; }
  // Verify peer
  long v = SSL_get_verify_result(_ssl);
  if (v != X509_V_OK) { fprintf(stderr, "TLS: verify_result=%ld\n", v); return false; }
  return true;
}

int BridgeSecureClient::connect(IPAddress ip, uint16_t port) {
  char host[32];
  snprintf(host, sizeof(host), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return connect(host, port);
}

int BridgeSecureClient::connect(const char *host, uint16_t port) {
  stop();

  struct addrinfo hints = {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  struct addrinfo* res = nullptr;
  char portStr[8]; snprintf(portStr, sizeof(portStr), "%u", port);
  if (getaddrinfo(host, portStr, &hints, &res) != 0) return 0;

  int sock = -1;
  for (struct addrinfo* p = res; p; p = p->ai_next) {
    sock = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (sock < 0) continue;
    if (::connect(sock, p->ai_addr, p->ai_addrlen) == 0) break;
    ::close(sock); sock = -1;
  }
  freeaddrinfo(res);
  if (sock < 0) return 0;

  if (!tlsConnect(sock, host)) { ::close(sock); return 0; }
  _sock = sock;
  _connected = true;
  return 1;
}

size_t BridgeSecureClient::write(uint8_t data) {
  return write(&data, 1);
}

size_t BridgeSecureClient::write(const uint8_t *buf, size_t size) {
  if (!_ssl) return 0;
  int n = SSL_write(_ssl, buf, (int)size);
  return (n > 0) ? (size_t)n : 0;
}

int BridgeSecureClient::available() {
  if (!_ssl) return 0;
  // Bytes buffered in SSL layer
  int p = SSL_pending(_ssl);
  if (p > 0) return p;
  // Fallback non-blocking peek
  char c;
  int n = ::recv(_sock, &c, 1, MSG_PEEK | MSG_DONTWAIT);
  return (n > 0) ? n : 0;
}

int BridgeSecureClient::read() {
  uint8_t b;
  int n = read(&b, 1);
  return (n == 1) ? (int)b : -1;
}

int BridgeSecureClient::read(uint8_t *buf, size_t size) {
  if (!_ssl) return -1;
  int n = SSL_read(_ssl, buf, (int)size);
  return (n > 0) ? n : -1;
}

void BridgeSecureClient::stop() {
  if (_ssl) { SSL_shutdown(_ssl); SSL_free(_ssl); _ssl = nullptr; }
  if (_ctx) { SSL_CTX_free(_ctx); _ctx = nullptr; }
  if (_sock >= 0) { ::close(_sock); _sock = -1; }
  _connected = false;
}

#endif // __linux__
