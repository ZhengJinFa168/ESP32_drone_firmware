#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include "core/eventService/EventType.h"
#include "core/Log.h"
#include "config/ArmingConfig.h"
#include "WebUI.h"
#include "WebSocketRcPacket.h"

class WebSocketRcProvider {
private:
    AsyncWebServer server;
    AsyncWebSocket ws;

    // Length-1 "mailbox" queue holding the latest RC packet.
    // onWsEvent runs on the async_tcp task, getRcData on loopTask, so the
    // handoff must be thread-safe; the queue does the locking and copying.
    // Statically allocated: the queue's control block and storage are members.
    StaticQueue_t _rcQueueControl;
    uint8_t _rcQueueStorage[sizeof(RcInputData)];
    QueueHandle_t _rcQueue = nullptr;

    // Malformed packets (written on async_tcp, read on loopTask)
    std::atomic<uint32_t> _malformedPackets{0};

    void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if (type == WS_EVT_DATA) {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                // Strict parse into a bounded copy (never writes into the frame buffer).
                // Ranges are checked by RcInputModule for every provider.
                RcInputData parsed;
                if (parseWebSocketRcPacket(reinterpret_cast<const char*>(data), len, parsed)) {
                    xQueueOverwrite(_rcQueue, &parsed); // Replaces any unread packet, never blocks
                } else {
                    _malformedPackets++;  // Not logged here: logging belongs to the loop task
                }
            }
        }
    }

public:
    WebSocketRcProvider() : server(80), ws("/ws") {}

    void setup() {
        // Create the mailbox before any WebSocket handler can write to it
        _rcQueue = xQueueCreateStatic(1, sizeof(RcInputData), _rcQueueStorage, &_rcQueueControl);
        if (!_rcQueue) {
            LOG_E("WS_RC", "failed to create RC queue, RC input disabled");
            return;
        }

        // Start WiFi Access Point
        LOG_I("WS_RC", "starting WiFi AP...");
        WiFi.softAP("ESP32_Drone", "password123");
        LOG_I("WS_RC", "AP IP address: %s", WiFi.softAPIP().toString().c_str());

        // Setup WebSocket
        ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
            this->onWsEvent(server, client, type, arg, data, len);
        });
        server.addHandler(&ws);

        // Serve HTML
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send_P(200, "text/html", WEB_UI_HTML);
        });

        server.begin();
        LOG_I("WS_RC", "web server started");
    }

    bool getRcData(RcInputData& outData) {
        // ws.cleanupClients() should be called periodically to free up memory from disconnected clients
        ws.cleanupClients(); 

        if (!_rcQueue) return false;

        // Non-blocking: true only when a new packet arrived since the last call
        return xQueueReceive(_rcQueue, &outData, 0) == pdTRUE;
    }

    // Total packets that did not match the packet format (since boot)
    uint32_t malformedPackets() const { return _malformedPackets.load(); }
};

