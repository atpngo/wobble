#include "communication.h"

void Communication::init()
{
    WiFi.mode(WIFI_STA);
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESPNow init failed");
        return;
    }
}

bool Communication::addPeer(const uint8_t *mac)
{
    esp_now_peer_info_t p{};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 0;
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
    return esp_now_send(mac, (uint8_t *)&pkt, pkt.len) == ESP_OK;
}

void Communication::broadcast(const Packet &pkt)
{
    for (auto &p : _peers)
    {
        esp_now_send(p.peer_addr, (uint8_t *)&pkt, sizeof(Packet));
    }
}

void Communication::onReceive(esp_now_recv_cb_t cb)
{
    esp_now_register_recv_cb(cb);
}