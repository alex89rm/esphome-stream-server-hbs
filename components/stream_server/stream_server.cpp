#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <vector>
#include <algorithm>
#include <errno.h>

// ESP-IDF RMT/GPIO
#include "driver/rmt.h"
#include "driver/gpio.h"

static const char *TAG = "stream_server";

using namespace esphome;

// -----------------------------
// HBS via RMT (ESP-IDF legacy)
// -----------------------------
#define HBS_TX_GPIO          GPIO_NUM_17  // <-- cambia qui il pin di uscita HBS
#define HBS_CLK_DIV          80           // 80MHz / 80 = 1 tick = 1 µs
#define HBS_BIT_US           104
#define HBS_HALF_BIT_US      52

static bool g_hbs_inited = false;

static inline rmt_item32_t hbs_item(int level0, int dur0, int level1, int dur1) {
  rmt_item32_t it;
  it.level0 = level0;
  it.duration0 = dur0;
  it.level1 = level1;
  it.duration1 = dur1;
  return it;
}

// Codifica un byte in simboli RMT secondo lo schema HBS.
// Ritorna il numero di item scritti nel vettore.
static size_t hbs_encode_byte(uint8_t data, std::vector<rmt_item32_t> &items) {
  size_t before = items.size();

  // Start: 52us LOW + 52us HIGH
  items.push_back(hbs_item(0, HBS_HALF_BIT_US, 1, HBS_HALF_BIT_US));

  // Dati LSB first + conteggio parità
  int ones = 0;
  for (int i = 0; i < 8; i++) {
    int bit = (data >> i) & 1;
    if (bit) {
      // 1 → 104us HIGH
      items.push_back(hbs_item(1, HBS_BIT_US, 1, 0));
      ones++;
    } else {
      // 0 → 52us LOW + 52us HIGH
      items.push_back(hbs_item(0, HBS_HALF_BIT_US, 1, HBS_HALF_BIT_US));
    }
  }

  // Parità even
  bool parity_one = (ones % 2) != 0; // se #1 è dispari, parità deve essere 1
  if (parity_one) {
    items.push_back(hbs_item(1, HBS_BIT_US, 1, 0));
  } else {
    items.push_back(hbs_item(0, HBS_HALF_BIT_US, 1, HBS_HALF_BIT_US));
  }

  // Stop: 104us HIGH
  items.push_back(hbs_item(1, HBS_BIT_US, 1, 0));

  return items.size() - before;
}

static void hbs_init() {
  if (g_hbs_inited) return;

  // Config RMT TX su HBS_TX_GPIO
  rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(HBS_TX_GPIO, RMT_CHANNEL_0);
  cfg.clk_div = HBS_CLK_DIV;                 // 1 tick = 1 µs
  cfg.tx_config.idle_output_en = true;       // mantieni livello idle
  cfg.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH; // idle HIGH

  ESP_ERROR_CHECK(rmt_config(&cfg));
  ESP_ERROR_CHECK(rmt_driver_install(cfg.channel, 0, 0));

  // Porta il pin HIGH (idle) anche lato GPIO
  gpio_config_t io = {
    .pin_bit_mask = (1ULL << HBS_TX_GPIO),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io);
  gpio_set_level(HBS_TX_GPIO, 1);

  g_hbs_inited = true;
}

// Invio non bloccante: codifica 'len' byte in un buffer di item e lancia rmt_write_items(..., wait=false)
static void hbs_send_bytes(const uint8_t *data, size_t len) {
  if (!g_hbs_inited) hbs_init();

  // Ogni byte → 1 start + 8 data + 1 parity + 1 stop = 11 item
  // (usiamo std::vector per semplicità)
  std::vector<rmt_item32_t> items;
  items.reserve(len * 12);

  for (size_t i = 0; i < len; i++) {
    hbs_encode_byte(data[i], items);
  }

  // Non bloccante
  // NOTE: se invii chiamate in rapida successione potresti voler
  // controllare lo stato della coda o usare un mutex.
  ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL_0, items.data(), items.size(), false));
}

// -----------------------------
// StreamServer originale
// -----------------------------

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // init HBS (aggiunta)
    hbs_init();

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

    struct sockaddr_storage bind_addr;
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2023, 4, 0)
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
#else
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), htons(this->port_));
#endif

    this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
    this->socket_->setblocking(false);
    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    this->accept();
    this->read();
    this->flush();
    this->write();
    this->cleanup();
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address().c_str(), this->port_);
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
#ifdef USE_SENSOR
    LOG_SENSOR("  ", "Connection count:", this->connection_count_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_)
        client.socket->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(this->clients_.size() > 0);
#endif
#ifdef USE_SENSOR
    if (this->connection_count_sensor_)
        this->connection_count_sensor_->publish_state(this->clients_.size());
#endif
}

void StreamServerComponent::accept() {
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier, this->buf_head_);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
    this->publish_sensor();
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    if (last_client != this->clients_.end()) {
        this->clients_.erase(last_client, this->clients_.end());
        this->publish_sensor();
    }
}

void StreamServerComponent::read() {
    size_t len = 0;
    int available;
    while ((available = this->stream_->available()) > 0) {
        size_t free = this->buf_size_ - (this->buf_head_ - this->buf_tail_);
        if (free == 0) {
            // Only overwrite if nothing has been added yet, otherwise give flush() a chance to empty the buffer first.
            if (len > 0)
                return;

            ESP_LOGE(TAG, "Incoming bytes available, but outgoing buffer is full: stream will be corrupted!");
            free = std::min<size_t>(available, this->buf_size_);
            this->buf_tail_ += free;
            for (Client &client : this->clients_) {
                if (client.position < this->buf_tail_) {
                    ESP_LOGW(TAG, "Dropped %u pending bytes for client %s", this->buf_tail_ - client.position, client.identifier.c_str());
                    client.position = this->buf_tail_;
                }
            }
        }

        // Fill all available contiguous space in the ring buffer.
        len = std::min<size_t>(available, std::min<size_t>(this->buf_ahead(this->buf_head_), free));
        this->stream_->read_array(&this->buf_[this->buf_index(this->buf_head_)], len);
        this->buf_head_ += len;
    }
}

void StreamServerComponent::flush() {
    ssize_t written;
    this->buf_tail_ = this->buf_head_;
    for (Client &client : this->clients_) {
        if (client.disconnected || client.position == this->buf_head_)
            continue;

        // Split the write into two parts: from the current position to the end of the ring buffer, and from the start
        // of the ring buffer until the head. The second part might be zero if no wraparound is necessary.
        struct iovec iov[2];
        iov[0].iov_base = &this->buf_[this->buf_index(client.position)];
        iov[0].iov_len = std::min(this->buf_head_ - client.position, this->buf_ahead(client.position));
        iov[1].iov_base = &this->buf_[0];
        iov[1].iov_len = this->buf_head_ - (client.position + iov[0].iov_len);
        if ((written = client.socket->writev(iov, 2)) > 0) {
            client.position += written;
        } else if (written == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;  // don't consider this client when calculating the tail position
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) transmit buffer is full, nothing to do.
        } else {
            ESP_LOGE(TAG, "Failed to write to client %s with error %d!", client.identifier.c_str(), errno);
        }

        this->buf_tail_ = std::min(this->buf_tail_, client.position);
    }
}

// --- MODIFICATA: TCP -> HBS su GPIO via RMT (non bloccante)
void StreamServerComponent::write() {
    uint8_t buf[128];
    ssize_t read;
    for (Client &client : this->clients_) {
        if (client.disconnected)
            continue;

        while ((read = client.socket->read(&buf, sizeof(buf))) > 0) {
            // Trasmette i byte ricevuti dal client TCP usando HBS via RMT
            hbs_send_bytes(buf, static_cast<size_t>(read));
        }

        if (read == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) receive buffer is empty, nothing to do.
        } else {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d!", client.identifier.c_str(), errno);
        }
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position)
    : socket(std::move(socket)), identifier{identifier}, position{position} {}
