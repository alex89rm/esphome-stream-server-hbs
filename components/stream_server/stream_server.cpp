#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include "driver/rmt_tx.h"

using namespace esphome;

static const char *TAG = "stream_server";

// --- HBS config ---
#define HBS_GPIO_PIN 17
#define HBS_BIT_US   104
#define HBS_HALF_US  52
#define RMT_RESOLUTION_HZ 1000000

static rmt_channel_handle_t hbs_channel = nullptr;

namespace esphome {
namespace stream_server {

// Client constructor
StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket,
                                      std::string identifier, size_t position)
    : socket(std::move(socket)), identifier{identifier}, position{position} {}

// --- HBS / RMT helpers ---
void init_hbs_rmt() {
    if (hbs_channel != nullptr) return;

    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num = (gpio_num_t)HBS_GPIO_PIN;
    tx_chan_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 4;
    tx_chan_config.flags.with_dma = false;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &hbs_channel));
    ESP_ERROR_CHECK(rmt_enable(hbs_channel));
}

static void hbs_send_byte(uint8_t data) {
    rmt_symbol_word_t symbols[32];
    int idx = 0;

    // Start bit
    symbols[idx].level0 = 0; symbols[idx].duration0 = HBS_HALF_US;
    symbols[idx].level1 = 1; symbols[idx].duration1 = HBS_HALF_US;
    idx++;

    int ones = 0;
    // Data bits LSB first
    for (int i = 0; i < 8; i++) {
        bool bit = data & (1 << i);
        if (bit) {
            symbols[idx].level0 = 1; symbols[idx].duration0 = HBS_BIT_US;
            symbols[idx].level1 = 1; symbols[idx].duration1 = 0;
            ones++;
        } else {
            symbols[idx].level0 = 0; symbols[idx].duration0 = HBS_HALF_US;
            symbols[idx].level1 = 1; symbols[idx].duration1 = HBS_HALF_US;
        }
        idx++;
    }

    // Parità even
    bool parity = (ones % 2) ? 1 : 0;
    if (parity) {
        symbols[idx].level0 = 1; symbols[idx].duration0 = HBS_BIT_US;
        symbols[idx].level1 = 1; symbols[idx].duration1 = 0;
    } else {
        symbols[idx].level0 = 0; symbols[idx].duration0 = HBS_HALF_US;
        symbols[idx].level1 = 1; symbols[idx].duration1 = HBS_HALF_US;
    }
    idx++;

    // Stop bit
    symbols[idx].level0 = 1; symbols[idx].duration0 = HBS_BIT_US;
    symbols[idx].level1 = 1; symbols[idx].duration1 = 0;
    idx++;

    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(hbs_channel, symbols, idx * sizeof(rmt_symbol_word_t), &tx_config));
}

// --- StreamServerComponent methods ---
void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // Setup RMT HBS
    init_hbs_rmt();

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
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_)
        client.socket->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {}

void StreamServerComponent::accept() {
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket) return;

    socket->setblocking(false);
    std::string identifier = socket->getpeername();
    this->clients_.emplace_back(std::move(socket), identifier, this->buf_head_);
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    if (last_client != this->clients_.end()) {
        this->clients_.erase(last_client, this->clients_.end());
    }
}

void StreamServerComponent::read() {
    size_t len = 0;
    int available;
    while ((available = this->stream_->available()) > 0) {
        size_t free = this->buf_size_ - (this->buf_head_ - this->buf_tail_);
        if (free == 0) return;

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

        struct iovec iov[2];
        iov[0].iov_base = &this->buf_[this->buf_index(client.position)];
        iov[0].iov_len = std::min(this->buf_head_ - client.position, this->buf_ahead(client.position));
        iov[1].iov_base = &this->buf_[0];
        iov[1].iov_len = this->buf_head_ - (client.position + iov[0].iov_len);
        if ((written = client.socket->writev(iov, 2)) > 0) {
            client.position += written;
        } else if (written == 0 || errno == ECONNRESET) {
            client.disconnected = true;
            continue;
        }
    }
}

// --- write con HBS ---
void StreamServerComponent::write() {
    uint8_t buf[128];
    ssize_t read;

    for (Client &client : this->clients_) {
        if (client.disconnected) continue;

        while ((read = client.socket->read(&buf, sizeof(buf))) > 0) {
            for (int i = 0; i < read; i++)
                hbs_send_byte(buf[i]);
        }

        if (read == 0 || errno == ECONNRESET) {
            client.disconnected = true;
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d!", client.identifier.c_str(), errno);
        }
    }
}

}  // namespace stream_server
}  // namespace esphome
