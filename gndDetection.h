/* 
 * file: gndDetection.h
 * author: J. van Hooydonk
 * comments: Marklin ground detection for PCB of G. Giebens https://github.com/GeertGiebens
 *
 * revision history:
 *  v0.1 Creation (22/06/2025)
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GNDDETECTION_H
#define	GNDDETECTION_H

#include "config.h"

// definitions
#define POWER_ON_DELAY 10
#define POWER_OFF_DELAY 20
#define DEBOUNCE_DELAY_0 10
#define DEBOUNCE_DELAY_1 200
#define THRESHOLD_LOW 0x0080
#define THRESHOLD_HIGH 0x03e0

// ground detection callback definition (as function pointer)
typedef void (*gndDetectionCallback_t)(uint8_t, bool);

// variables
// ground detection used variables
gndDetectionCallback_t gndDetectionCallback;
uint8_t gndDetectionInputIndex;
uint16_t adcResults[16];
bool powerState;
uint16_t delayPowerOn;
uint16_t delayPowerOff;
bool oldGndDetectionInputState[16];
bool gndDetectionInputUpdateRequired[16];
uint16_t debounceState0[16];
uint16_t debounceState1[16];
uint8_t channels[16] = {0x1a, 0x1b, 0x14, 0x15, 0x1c, 0x1d, 0x1e, 0x1f, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

// routines
void gndDetectionInit(gndDetectionCallback_t);
void gndDetectionInputInit(void);
void gndDetectionInitAdc(void);
bool getPowerState(void);
void sampleGndDetectionInput(void);
void handleGndDetectionInputs(void);
void startAdc(uint8_t);
void dischargeSampleCap(void);
bool scanInputs(bool);
void updateGndDetectionInput(uint8_t, bool);

#endif	/* GNDDETECTION_H */
