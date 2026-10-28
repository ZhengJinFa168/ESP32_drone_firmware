#pragma once
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "core/eventService/EventType.h"
#include "config/ArmingConfig.h"
#include "WebUI.h"

class WebSocketRcProvider {
private:
    AsyncWebServer server;
    AsyncWebSocket ws;

    // Length-1 "mailbox" queue holding the latest RC packet.
    // onWsEvent runs on the async_tcp task, getRcData on loopTask, so the
    // handoff must be thread-safe; the queue does the locking and copying.
    QueueHandle_t _rcQueue = nullptr;

    void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        if (type == WS_EVT_DATA) {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0; // Null-terminate
                char* msg = (char*)data;
                
                // Parse "T:0.00,Y:0.00,P:0.00,R:0.00,A:0" (A = arm switch, 0/1)
                RcInputData parsed = {0.0f, 0.0f, 0.0f, 0.0f, {0, 0, 0, 0}, false};
                int armSwitch = 0;
                if (sscanf(msg, "T:%f,Y:%f,P:%f,R:%f,A:%d",
                           &parsed.throttle, &parsed.yaw, &parsed.pitch, &parsed.roll, &armSwitch) == 5) {
                    parsed.aux[ARM_AUX_CHANNEL] = armSwitch ? 1 : 0;
                    parsed.linkOk = true;
                    xQueueOverwrite(_rcQueue, &parsed); // Replaces any unread packet, never blocks
                }
            }
        }
    }

public:
    WebSocketRcProvider() : server(80), ws("/ws") {}

    void setup() {
        // Create the mailbox before any WebSocket handler can write to it
        _rcQueue = xQueueCreate(1, sizeof(RcInputData));
        if (!_rcQueue) {
            Serial.println("Error: failed to create RC queue. RC input disabled.");
            return;
        }

        // Start WiFi Access Point
        Serial.println("Starting WiFi AP...");
        WiFi.softAP("ESP32_Drone", "password123");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());

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
        Serial.println("Web Server started");
    }

    bool getRcData(RcInputData& outData) {
        // ws.cleanupClients() should be called periodically to free up memory from disconnected clients
        ws.cleanupClients(); 

        if (!_rcQueue) return false;

        // Non-blocking: true only when a new packet arrived since the last call
        return xQueueReceive(_rcQueue, &outData, 0) == pdTRUE;
    }
};

