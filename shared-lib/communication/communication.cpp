#include "communication.h"

static Communication *_instance = nullptr;

void Communication::init()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESPNow init failed");
        return;
    }
    _instance = this;
    esp_now_register_recv_cb(_espnowRecv);
}

bool Communication::addPeer(const uint8_t *mac)
{
    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, mac, 6);
    p.encrypt = false;
    if (esp_now_add_peer(&p) == ESP_OK)
    {
        _peers.push_back(p);
        return true;
    }
    return false;
}

bool Communication::send(const uint8_t *mac, const Packet &pkt)
{
    return esp_now_send(mac, (uint8_t *)&pkt, sizeof(Packet)) == ESP_OK;
}

void Communication::broadcast(const Packet &pkt)
{
    for (auto &p : _peers)
    {
        esp_now_send(p.peer_addr, (uint8_t *)&pkt, sizeof(Packet));
    }
}

void Communication::onReceive(RecvCallback cb)
{
    _onRecv = cb;
}

void Communication::_espnowRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    if (!_instance || len < (int)sizeof(Packet))
        return;
    Packet pkt;
    memcpy(&pkt, data, sizeof(Packet));
    if (_instance->_onRecv)
        _instance->_onRecv(esp_now_info, pkt);
}
