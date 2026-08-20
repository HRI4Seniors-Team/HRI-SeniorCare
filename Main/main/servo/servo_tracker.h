#ifndef RESONA_SERVO_TRACKER_H_
#define RESONA_SERVO_TRACKER_H_

#include "uart_k210.h"

void InitializeServoSelfTest();
void ServoTrackerOnVisionPacket(const VisionEmotionPacket& pkt);

#endif  // RESONA_SERVO_TRACKER_H_
