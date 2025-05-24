#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <functional>

enum class Command : uint8_t
{
    NOP,
    FORWARD,
    BACKWARD,
    TURN_L,
    TURN_R
};
struct Telemetry
{
    int32_t left_enc, right_enc;
    float pitch;
};

// max size per ESP-NOW packet
static constexpr size_t MAX_PAYLOAD_LEN = 250;

// message types
enum class Message : uint8_t
{
    Telemetry = 0,
    Command = 1,
    // add more types as needed
};

// generic header + payload
struct Packet
{
    Message type;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD_LEN];
} __attribute__((packed));

class Communication
{
public:
    using RecvCallback = std::function<void(const esp_now_recv_info_t *info, const Packet &)>;

    // initialize WiFi + ESP-NOW
    void init();

    // register a peer by MAC address
    bool addPeer(const uint8_t *mac);

    // send a packet to a single peer
    bool send(const uint8_t *mac, const Packet &pkt);

    // send a packet to all registered peers
    void broadcast(const Packet &pkt);

    // set a callback for incoming packets
    void onReceive(RecvCallback cb);

private:
    std::vector<esp_now_peer_info_t> _peers;
    RecvCallback _onRecv = nullptr;

    static void _espnowRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len);
};

#endif // COMMUNICATION_H