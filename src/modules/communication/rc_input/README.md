### What was added:
1.  **[platformio.ini](file:///Users/luigizheng/Desktop/ESP32_drone_firmware/platformio.ini)**: Added the `mathieucarbou/ESPAsyncWebServer` dependency to handle the async web serving.
2.  **[RcInputModule.h](file:///Users/luigizheng/Desktop/ESP32_drone_firmware/src/modules/communication/rc_input/RcInputModule.h)**: The main module orchestrator. Notice the `#define USE_WEBSOCKET_RC` at the top. This is the zero-overhead switch you'll use later to swap to CRSF.
3.  **[WebSocketRcProvider.h](file:///Users/luigizheng/Desktop/ESP32_drone_firmware/src/modules/communication/rc_input/WebSocketRcProvider.h)**: The provider that spins up the "ESP32_Drone" WiFi network and parses the incoming WebSocket data into `RcInputData`.
4.  **[WebUI.h](file:///Users/luigizheng/Desktop/ESP32_drone_firmware/src/modules/communication/rc_input/WebUI.h)**: The 100% offline HTML/JS payload containing the touch canvas joystick logic.
5.  **[main.cpp](file:///Users/luigizheng/Desktop/ESP32_drone_firmware/src/main.cpp)**: Registered the module with the main `drone` app.

### How to Test It:
1.  Click **Upload** in PlatformIO to flash the drone.
2.  Grab your iPhone and open WiFi settings. Connect to **`ESP32_Drone`** (Password: `password123`).
3.  Open Safari and navigate to **`http://192.168.4.1`**.
4.  The two virtual thumbsticks will appear. As you drag them around, they instantly send WebSocket packets to the drone at 50 frames per second.
5.  Check your Serial Monitor on your computer; you'll see the connection logs and (if you log the `RC_INPUT` event later) the values changing!
