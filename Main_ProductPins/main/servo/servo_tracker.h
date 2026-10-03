#ifndef RESONA_SERVO_TRACKER_H_
#define RESONA_SERVO_TRACKER_H_

#include "uart_k210.h"

void InitializeServoSelfTest();
void ServoTrackerOnVisionPacket(const VisionEmotionPacket& pkt);
uint32_t ServoTrackerLastCommandUs();
uint32_t ServoTrackerMoveCount();

#endif  // RESONA_SERVO_TRACKER_H_
