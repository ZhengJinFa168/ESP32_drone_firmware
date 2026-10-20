#pragma once
#include <Arduino.h>

const char WEB_UI_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0">
    <title>Drone RC</title>
    <style>
        body, html { margin: 0; padding: 0; height: 100%; background: #222; touch-action: none; overflow: hidden; font-family: sans-serif; color: white; }
        .container { display: flex; height: 100%; width: 100%; }
        .half { flex: 1; display: flex; justify-content: center; align-items: center; position: relative; }
        .stick-container { width: 150px; height: 150px; background: rgba(255,255,255,0.1); border-radius: 50%; position: relative; }
        .stick { width: 60px; height: 60px; background: rgba(255,255,255,0.5); border-radius: 50%; position: absolute; top: 45px; left: 45px; pointer-events: none; }
        .status { position: absolute; top: 10px; width: 100%; text-align: center; font-size: 14px; pointer-events: none; z-index: 10; }
        #status-text { color: red; }
    </style>
</head>
<body>
    <div class="status">WebSocket: <span id="status-text">Disconnected</span></div>
    <div class="container">
        <!-- Left Stick: Throttle (Y) and Yaw (X) -->
        <div class="half" id="left-zone">
            <div class="stick-container" id="left-container">
                <div class="stick" id="left-stick"></div>
            </div>
        </div>
        <!-- Right Stick: Pitch (Y) and Roll (X) -->
        <div class="half" id="right-zone">
            <div class="stick-container" id="right-container">
                <div class="stick" id="right-stick"></div>
            </div>
        </div>
    </div>
    <script>
        let ws;
        const statusEl = document.getElementById('status-text');
        
        let rc = { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        
        function connect() {
            ws = new WebSocket('ws://' + window.location.host + '/ws');
            ws.onopen = () => { statusEl.innerText = 'Connected'; statusEl.style.color = 'lime'; };
            ws.onclose = () => { statusEl.innerText = 'Disconnected'; statusEl.style.color = 'red'; setTimeout(connect, 1000); };
        }
        connect();
        
        // 50Hz send loop
        setInterval(() => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(`T:${rc.throttle.toFixed(2)},Y:${rc.yaw.toFixed(2)},P:${rc.pitch.toFixed(2)},R:${rc.roll.toFixed(2)}`);
            }
        }, 20);

        function setupStick(zoneId, stickId, containerId, isLeft) {
            const zone = document.getElementById(zoneId);
            const stick = document.getElementById(stickId);
            const container = document.getElementById(containerId);
            const maxRadius = 75; // 150px / 2
            
            let activeTouch = null;
            let centerX = 0, centerY = 0;

            function updateStickPos(x, y) {
                stick.style.transform = `translate(${x}px, ${y}px)`;
            }

            zone.addEventListener('touchstart', (e) => {
                e.preventDefault();
                if (activeTouch !== null) return;
                
                let rect = container.getBoundingClientRect();
                centerX = rect.left + maxRadius;
                centerY = rect.top + maxRadius;
                
                const touch = e.changedTouches[0];
                activeTouch = touch.identifier;
                handleMove(touch);
            }, {passive: false});

            zone.addEventListener('touchmove', (e) => {
                e.preventDefault();
                for (let i = 0; i < e.changedTouches.length; i++) {
                    if (e.changedTouches[i].identifier === activeTouch) {
                        handleMove(e.changedTouches[i]);
                        break;
                    }
                }
            }, {passive: false});

            function endTouch(e) {
                for (let i = 0; i < e.changedTouches.length; i++) {
                    if (e.changedTouches[i].identifier === activeTouch) {
                        activeTouch = null;
                        
                        let resetX = 0;
                        let resetY = 0;
                        
                        if (isLeft) {
                            resetY = maxRadius; // Bottom
                            rc.throttle = 0.0;
                            rc.yaw = 0.0;
                        } else {
                            rc.pitch = 0.0;
                            rc.roll = 0.0;
                        }
                        
                        updateStickPos(resetX, resetY);
                        break;
                    }
                }
            }

            zone.addEventListener('touchend', endTouch);
            zone.addEventListener('touchcancel', endTouch);

            function handleMove(touch) {
                let dx = touch.clientX - centerX;
                let dy = touch.clientY - centerY;
                
                let distance = Math.sqrt(dx*dx + dy*dy);
                if (distance > maxRadius) {
                    dx = dx * (maxRadius / distance);
                    dy = dy * (maxRadius / distance);
                }
                
                updateStickPos(dx, dy);
                
                let normX = dx / maxRadius;
                let normY = -dy / maxRadius; // Invert Y so up is positive
                
                if (isLeft) {
                    rc.yaw = normX;
                    rc.throttle = (normY + 1.0) / 2.0; 
                } else {
                    rc.roll = normX;
                    rc.pitch = normY;
                }
            }
            
            if (isLeft) {
                updateStickPos(0, maxRadius); // Throttle at bottom
            } else {
                updateStickPos(0, 0); // Center
            }
        }
        
        setupStick('left-zone', 'left-stick', 'left-container', true);
        setupStick('right-zone', 'right-stick', 'right-container', false);
    </script>
</body>
</html>
)=====";

