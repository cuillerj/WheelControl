/*
I developed a library for Arduino that is free of usage.
This software has been developped for ATmega2560 
Software is designed to handle from 1 to 4 decoders simultaneously. It works asynchronously of the main code. 
The main code has to initialize some parameters and then can start the encoder control independently of each others . 
When a requested threshold expressed in number of holes will be reached  the main code will be interrupt by the library code and will be able to act accordingly.

WheelControl () is used to define the object (before the setup())
	for each x encoder
		wheelIdxEncoderHoles must contain the number of holes of the x encoder
		wheelIdxIncoderHighValue must contain the threshold  (result of analogRead of the x encoder output) over that encoder input is high (meaning IR receiver can not see IR emitter)
		wheelIdxIncoderLowValue must contain the threshold  (result of analogRead of the x encoder output) under that encoder input is low (meaning IR receiver can not see IR emitter)
		wheelIdxAnalogEncoderInput must contain the analog Arduino input connected to the x encoder output
		set 0 if not used
	wheelPinInterrupt must contain the software PIN used to interrupt main code (look at the Arduino documention to choose one)
	delayMiniBetweenHoles must contain the minimum duration in micro seconds between two encoder holes at the maximum speed 
		for instance if the maximim wheel RPM is 120 and the number of holes encoder is 8 it means: 2 turns x 8 holes / second = 62.5 ms >> set delayMiniBetweenHoles below 62.5 => 50
	samplingRatio increase sampling frequency to adjust electrical characteristics VS signal as long high as low - too low we will lost holes too high we will consume unusefull CPU

StartWheelControl() is used to start encoder control
	for each x encoder
		wheelIdxControlOn true to activate the x encoder control / false not to use the x encoder control
		wheelIdxInterruptOn true to activate the interrupt for x encoder  / false not to use the interrupt for the x encoder control
			wheelIdxControlOn must be true if wheelIdxInterruptOn is true
		wheelIdxLimitation must contain the threshold number of holes that will provide the interrupt
			wheelIdxInterruptOn must be true if wheelIdxLimitation is not equal 0
	it is possible to call multiple time StartWheelControl() to start different encoder without previously stopping 
	
StopWheelControl() is used to stop encoder control	
	for each x encoder
		wheelIdxControlOn true to stop the x encoder control / false to let x encoder in the previous state
		it is possible to call multiple time StartWheelControl() to stop different encoder

GetWheelThreshold(x) will return the current x encoder threshold
ClearThreshold(x) is used to clear x encoder threshold
GetCurrentHolesCount(x) will return the current number of holes for the x encoder since the previous start
GetLastTurnSpeed(x) will return the last turnspeed for the x encoder
Get2LastTurnSpeed(x) will return the average last two turns speed for the x encoder
GetLastWheelInterruptId(x) will return the last encoder number that provide an interrupt
GetInstantTurnSpeed(x) will return the instant turn speed for the x encoder
GetTurnSpeed(x) will return the mitigate turn speed for the x encoder
GetWheeLowValue(x) will return the lowest analog value for the x encoder since the last StartWheelControl
GetWheeHighValue(x) will return the highest analog value for the x encoder since the last StartWheelControl

StartWheelPulse(pulseLimitation) act as a simple timer - pulseLimitation is the duration after that the main code will be interrupt.
	can be used to turn a little bit the motor (encoders are not used)
*/
#include <Arduino.h>
#ifndef WheelControl_h_included
#define WheelControl_h_included


class WheelControl
{
public:
WheelControl (
				uint8_t wheelId0EncoderHoles, int wheelId0IncoderHighValue, int wheelId0IncoderLowValue, int wheelId0AnalogEncoderInput,
				uint8_t wheelId1EncoderHoles, int wheelId1IncoderHighValue ,int wheelId1IncoderLowValue, int wheelId1AnalogEncoderInput, 
				uint8_t wheelId2EncoderHoles, int wheelId2IncoderHighValue ,int wheelId2IncoderLowValue, int wheelId2AnalogEncoderInput, 
				uint8_t wheelId3EncoderHoles, int wheelId3IncoderHighValue ,int wheelId3IncoderLowValue, int wheelId3AnalogEncoderInput, 
				uint8_t wheelPinInterrupt, float delayMiniBetweenHoles,float samplingRatio
			);
				
				
void StartWheelControl(
					boolean wheelId0ControlOn,boolean wheelId0InterruptOn,unsigned int wheelId0Limitation,
					boolean wheelId1ControlOn,boolean wheelId1InterruptOn,unsigned int wheelId1Limitation,
					boolean wheelId2ControlOn,boolean wheelId2InterruptOn,unsigned int wheelId2Limitation,
					boolean wheelId3ControlOn,boolean wheelId3InterruptOn,unsigned int wheelId3Limitation
			);
void StartWheelPulse(unsigned int pulseLimitation);
void StopWheelControl(boolean wheelId0ControlOn, boolean wheelId1ControlOn, boolean wheelId2ControlOn, boolean wheelId3ControlOn);
void ClearThreshold(uint8_t wheelId);
void IncreaseThreshold(uint8_t wheelId, unsigned int pulseIncrease );
unsigned int GetCurrentHolesCount(uint8_t wheelId);
unsigned int GetMinLevel(uint8_t  wheelId);
unsigned int GetMaxLevel(uint8_t  wheelId);
float GetLastTurnSpeed(uint8_t wheelId);
float Get2LastTurnSpeed(uint8_t wheelId);
float GetInstantTurnSpeed(uint8_t wheelId);
float GetTurnSpeed(uint8_t wheelId);
unsigned int GetWheelThreshold(uint8_t wheelId);
uint8_t GetLastWheelInterruptId();
unsigned int GetWheeLowValue(uint8_t wheelId); 
unsigned int GetWheeHighValue(uint8_t wheelId); 


};
    
#endif