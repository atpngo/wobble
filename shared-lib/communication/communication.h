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

#pragma pack(push, 1)
struct Telemetry
{
    float pitch;
    int left_enc;
    int right_enc;
};

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
    uint8_t payload[100];
};
#pragma pack(pop)

class Communication
{
public:
    // initialize WiFi + ESP-NOW
    void init();

    // register a peer by MAC address
    bool addPeer(const uint8_t *mac);

    // send a packet to a single peer
    bool send(const uint8_t *mac, const Packet &pkt);

    // send a packet to all registered peers
    void broadcast(const Packet &pkt);

    // set a callback for incoming packets
    void onReceive(esp_now_recv_cb_t cb);

private:
    std::vector<esp_now_peer_info_t> _peers;
};

#endif // COMMUNICATION_H