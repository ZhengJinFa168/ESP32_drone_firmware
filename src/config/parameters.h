#pragma once

// PID gains – roll
#define ROLL_KP     0.8f
#define ROLL_KI     0.05f
#define ROLL_KD     0.01f

// PID gains – pitch
#define PITCH_KP    0.8f
#define PITCH_KI    0.05f
#define PITCH_KD    0.01f

// PID gains – yaw
#define YAW_KP      1.0f
#define YAW_KI      0.1f
#define YAW_KD      0.0f

// Flight limits
#define MAX_ANGLE_DEG   35.0f
#define MAX_YAW_RATE    200.0f  // deg/s
