/*

*/
#include <Arduino.h>

#ifndef WheelControl_h_included
#define WheelControl_h_included
#define _sizeOfRevSpeedArray 8 //size of the array containing latest revolution wheel speed
//#define delayMiniBetweenHoles  750
class WheelControl
{
public:
WheelControl (
				uint8_t wheelId0EncoderHoles, int wheelId0IncoderHighValue, int wheelId0IncoderLowValue, int wheelId0AnalogEncoderInput,
				uint8_t wheelId1EncoderHoles, int wheelId1IncoderHighValue ,int wheelId1IncoderLowValue, int wheelId1AnalogEncoderInput, 
				uint8_t wheelId2EncoderHoles, int wheelId2IncoderHighValue ,int wheelId2IncoderLowValue, int wheelId2AnalogEncoderInput, 
				uint8_t wheelId3EncoderHoles, int wheelId3IncoderHighValue ,int wheelId3IncoderLowValue, int wheelId3AnalogEncoderInput, 
				uint8_t wheelPinInterrupt, int delayMiniBetweenHoles
			);
				
				
void StartWheelControl(
					boolean wheelId0ControlOn, boolean wheelId0InterruptOn,unsigned int wheelId0Limitation,
					boolean wheelId1ControlOn,boolean wheelId1InterruptOn,unsigned int wheelId1Limitation,
					boolean wheelId2ControlOn,boolean wheelId2InterruptOn,unsigned int wheelId2Limitation,
					boolean wheelId3ControlOn,boolean wheelId3InterruptOn,unsigned int wheelId3Limitation
			);
void StopWheelControl(boolean wheelId0ControlOn, boolean wheelId1ControlOn, boolean wheelId2ControlOn, boolean wheelId3ControlOn);
void ClearThershold(uint8_t wheelId);
unsigned int GetCurrentHolesCount(uint8_t wheelId);
unsigned int GetMinLevel(uint8_t  wheelId);
unsigned int GetMaxLevel(uint8_t  wheelId);
float GetLastTurnSpeed(uint8_t wheelId);
float Get2LastTurnSpeed(uint8_t wheelId);
uint8_t GetLastWheelInterruptId();


//unsigned int _currentHolesCount[4];
boolean _wheelControlOn[4];
boolean _controlOn=false;

};
    
#endif