#pragma once

#define DEBUG       1
#define LOOP_HZ     500
#define LOOP_US     (1000000 / LOOP_HZ)

#if DEBUG
  #define DBG(x) Serial.println(x)
#else
  #define DBG(x)
#endif
