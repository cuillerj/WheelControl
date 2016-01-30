/*

*/
#include <Arduino.h>
#include "WheelControl.h"
unsigned int tcntWheel=65000;   // used to init timer overflow for instance 49911 for 1s cycle (tous les 8/100eme s
volatile float lastTurnWheelSpeed[4];
volatile float last2TurnWheelSpeed[4];
//volatile unsigned int instantWheelRevSpeed[_sizeOfRevSpeedArray*4];
volatile unsigned long prevWheelInterrupt[4];
volatile unsigned long prevWheelIntTime[4];
volatile unsigned long wheelInterrupt[4];
volatile unsigned long saveWheelInterrupt[4];
volatile unsigned long readAnalogTimer[4];
volatile unsigned long timerInt[4];
volatile unsigned long timer2Int[4];
volatile unsigned long microInt[4];
volatile unsigned int wheelSpeedCount[4];
volatile boolean startHigh[4];
volatile boolean flagLow [4];   
volatile unsigned int minWheelLevel[4];
volatile unsigned int maxWheelLevel[4];
int wheelIdIncoderHighValue[4];
int wheelIdIncoderLowValue[4];
int  wheelIdAnalogEncoderInput[4];
int _delayMiniBetweenHoles;
unsigned int wheelIdLimitation[4];
uint8_t softPinInterrupt;
boolean wheelInterruptOn[4];
uint8_t lastWheelInterruptId;
int8_t wheelIdEncoderHoles[4];

WheelControl::WheelControl (
				uint8_t wheelId0EncoderHoles, int wheelId0IncoderHighValue, int wheelId0IncoderLowValue, int wheelId0AnalogEncoderInput,
				uint8_t wheelId1EncoderHoles, int wheelId1IncoderHighValue ,int wheelId1IncoderLowValue, int wheelId1AnalogEncoderInput, 
				uint8_t wheelId2EncoderHoles, int wheelId2IncoderHighValue ,int wheelId2IncoderLowValue, int wheelId2AnalogEncoderInput, 
				uint8_t wheelId3EncoderHoles, int wheelId3IncoderHighValue ,int wheelId3IncoderLowValue, int wheelId3AnalogEncoderInput, 
				uint8_t wheelPinInterrupt, int delayMiniBetweenHoles
				)
				
			{
			wheelIdEncoderHoles[0]=wheelId0EncoderHoles;
			wheelIdAnalogEncoderInput[0]=wheelId0AnalogEncoderInput;
			wheelIdIncoderHighValue[0]=wheelId0IncoderHighValue;
			wheelIdIncoderLowValue[0]=wheelId0IncoderLowValue;
			wheelIdEncoderHoles[1]=wheelId1EncoderHoles;
			wheelIdAnalogEncoderInput[1]=wheelId1AnalogEncoderInput;
			wheelIdIncoderHighValue[1]=wheelId1IncoderHighValue;
			wheelIdIncoderLowValue[1]=wheelId1IncoderLowValue;
			wheelIdEncoderHoles[2]=wheelId2EncoderHoles;
			wheelIdAnalogEncoderInput[2]=wheelId2AnalogEncoderInput;
			wheelIdIncoderHighValue[2]=wheelId2IncoderHighValue;
			wheelIdIncoderLowValue[2]=wheelId2IncoderLowValue;
			wheelIdEncoderHoles[3]=wheelId3EncoderHoles;
			wheelIdAnalogEncoderInput[3]=wheelId3AnalogEncoderInput;
			wheelIdIncoderHighValue[3]=wheelId3IncoderHighValue;
			wheelIdIncoderLowValue[3]=wheelId3IncoderLowValue;
			softPinInterrupt=wheelPinInterrupt;
			_delayMiniBetweenHoles=delayMiniBetweenHoles;
			}
void WheelControl::StartWheelControl(boolean wheelId0ControlOn, boolean wheelId0InterruptOn,unsigned int wheelId0Limitation,
						boolean wheelId1ControlOn,boolean wheelId1InterruptOn,unsigned int wheelId1Limitation,
						boolean wheelId2ControlOn,boolean wheelId2InterruptOn,unsigned int wheelId2Limitation,
						boolean wheelId3ControlOn,boolean wheelId3InterruptOn,unsigned int wheelId3Limitation
			)
			{
				_controlOn=true;
				 if (softPinInterrupt!=0)
					{
					pinMode(softPinInterrupt,OUTPUT);
					digitalWrite(softPinInterrupt,LOW);
					}
				if (wheelId0ControlOn)
				{
					_wheelControlOn[0]=true;
					wheelInterruptOn[0]=wheelId0InterruptOn;
					wheelIdLimitation[0]=wheelId0Limitation;
					lastTurnWheelSpeed[0] = 0;
					last2TurnWheelSpeed[0] = 0;
					wheelInterrupt[0] = 0;
					saveWheelInterrupt[0] = 0;
					flagLow[0]=false;
					minWheelLevel[0]=1023;
					maxWheelLevel[0]=0;
					if (analogRead(wheelIdAnalogEncoderInput[0])> wheelIdIncoderHighValue[0])
					{
						startHigh[0]=true;           
					}
					else 
					{
						startHigh[0]=false;
					}
					
				}
				if (wheelId1ControlOn)
				{
					_wheelControlOn[1]=true;
					wheelInterruptOn[1]=wheelId1InterruptOn;
					wheelIdLimitation[1]=wheelId1Limitation;
					lastTurnWheelSpeed[1] = 0;
					last2TurnWheelSpeed[1] = 0;
					wheelInterrupt[1] = 0;
					saveWheelInterrupt[1] = 0;
					flagLow[1]=false;
					minWheelLevel[1]=1023;
					maxWheelLevel[1]=0;
					if (analogRead(wheelIdAnalogEncoderInput[1])> wheelIdIncoderHighValue[1])  // do we started on high or low position
					{
						startHigh[1]=true;    
					}
					else 
					{
						startHigh[1]=false;
					}

				}
				if (wheelId2ControlOn)
				{
					_wheelControlOn[2]=true;
					wheelInterruptOn[2]=wheelId2InterruptOn;
					wheelIdLimitation[2]=wheelId2Limitation;
					lastTurnWheelSpeed[2] = 0;
					last2TurnWheelSpeed[2] = 0;
					wheelInterrupt[2] = 0;
					saveWheelInterrupt[2] = 0;
					flagLow[2]=false;
					minWheelLevel[2]=1023;
					maxWheelLevel[2]=0;
					if (analogRead(wheelIdAnalogEncoderInput[2])> wheelIdIncoderHighValue[2])
					{
						startHigh[2]=true;
					}
					else 
					{
						startHigh[2]=false;
					}
						
				}
				if (wheelId3ControlOn)
				{
					_wheelControlOn[3]=true;
					wheelInterruptOn[3]=wheelId1InterruptOn;
					wheelIdLimitation[3]=wheelId1Limitation;
					lastTurnWheelSpeed[3] = 0;
					last2TurnWheelSpeed[3] = 0;
					wheelInterrupt[3] = 0;
					saveWheelInterrupt[3] = 0;
					flagLow[3]=false;
					minWheelLevel[3]=1023;
					maxWheelLevel[3]=0;
					if (analogRead(wheelIdAnalogEncoderInput[3])> wheelIdIncoderHighValue[3])
					{
						startHigh[3]=true;
					}
					else 
					{
						startHigh[3]=false;
					}

				}
				noInterrupts(); // disable all interrupts
				TCCR3A = 0;  // set entire TCCR4A register to 0
				TCCR3B = 0;  // set entire TCCR4B register to 0
				TCNT3 = tcntWheel; // 
				TCCR3B |= ((1 << CS12) ); // 256 prescaler
				TIMSK3 |= (1 << TOIE3); // enable timer overflow interrupt
				interrupts(); // enable all interrupts
			}
void WheelControl::StopWheelControl(boolean wheelId0ControlOn,	boolean wheelId1ControlOn,boolean wheelId2ControlOn,boolean wheelId3ControlOn)
			{
				_controlOn=false;
				if (wheelId0ControlOn)
				{
					_wheelControlOn[0]=false;
					wheelInterruptOn[0]=false;
					wheelIdLimitation[0]=0;
				}
				if (wheelId1ControlOn)
				{
					_wheelControlOn[1]=false;
					wheelInterruptOn[1]=false;
					wheelIdLimitation[1]=0;
				}
				if (wheelId2ControlOn)
				{
					_wheelControlOn[2]=false;
					wheelInterruptOn[2]=false;
					wheelIdLimitation[2]=0;
				}
				if (wheelId3ControlOn)
				{
					_wheelControlOn[3]=false;
					wheelInterruptOn[3]=false;
					wheelIdLimitation[3]=0;
				}
				
			}
unsigned int  WheelControl::GetCurrentHolesCount(uint8_t wheelId)
	{
	return wheelInterrupt[wheelId];
	}
unsigned int  WheelControl::GetMinLevel(uint8_t wheelId)
	{
	return minWheelLevel[wheelId];
	}
unsigned int  WheelControl::GetMaxLevel(uint8_t wheelId)
	{
	return maxWheelLevel[wheelId];
	}
float WheelControl::GetLastTurnSpeed(uint8_t wheelId)
	{
	return lastTurnWheelSpeed[wheelId];
	}
float WheelControl::Get2LastTurnSpeed(uint8_t wheelId)
	{
	return last2TurnWheelSpeed[wheelId];
	}
uint8_t WheelControl::GetLastWheelInterruptId()
	{
	return(lastWheelInterruptId);
	}
void WheelControl::ClearThershold(uint8_t wheelId)
{

					wheelInterruptOn[wheelId]=false;

}
ISR(TIMER3_OVF_vect)        // timer interrupt used to regurarly check rotation
	{
	TCNT3 = tcntWheel;            // preload timer to adjust duration
	for (int i=0;i<4;i++)
	{
		 int level = analogRead(wheelIdAnalogEncoderInput[i]);
		 if (level<minWheelLevel[i])
		 {
			minWheelLevel[i]=level;
		 }
		 if (level>maxWheelLevel[i])
		 {
			maxWheelLevel[i]=level;
		 }

	//	 readAnalogTimer[i] = micros();
	if (millis()-microInt[i]> _delayMiniBetweenHoles)    // if delay not big enough do nothing to avoid misreading
	{
		boolean switchOn;
		 if (level > wheelIdIncoderHighValue[i] && startHigh[i]==true)    // started high and high again 
		 {
			 switchOn=true;          
		 }
 		 if (level > wheelIdIncoderHighValue[i] && startHigh[i]==false)  // started high and low now 
		 {
			 switchOn=false;
		 }
		 if (level < wheelIdIncoderLowValue[i] && startHigh[i]==false)    // started low and low again 
		 {
			 switchOn=true;
		 }
 		 if (level < wheelIdIncoderLowValue[i] && startHigh[i]==true)     // started low and high now
		 {
			 switchOn=false;
		 }
		if (switchOn==true && flagLow[i] == true)  // one new hole detected
			{
			microInt[i]=millis();
			flagLow[i] = false;
			if (wheelInterrupt[i]%wheelIdEncoderHoles[i]==0)  // get time for the first occurence
			{
				timerInt[i] = millis();
			}
			if (wheelInterrupt[i]%(2*wheelIdEncoderHoles[i])==0)  // get time for the first occurence
			{
				timer2Int[i] = millis();
			}
			if (wheelInterrupt[i]%wheelIdEncoderHoles[i]==(wheelIdEncoderHoles[i]-1))  // get time for the last occurence
			{
				unsigned int deltaTime = millis() - timerInt[i];
				lastTurnWheelSpeed[i]=float (1000*(wheelIdEncoderHoles[i]-1)/wheelIdEncoderHoles[i])/deltaTime;
				timerInt[i] = millis();
			}
			if (wheelInterrupt[i]%(2*wheelIdEncoderHoles[i])==((2*wheelIdEncoderHoles[i])-1))  // get time for the last occurence
			{
				unsigned int deltaTime = millis() - timer2Int[i];
				last2TurnWheelSpeed[i]=float (1000*((2*wheelIdEncoderHoles[i])-1)*2/(2*wheelIdEncoderHoles[i]))/deltaTime;
				timer2Int[i] = millis();

			}
	//		wheelSpeedCount[i]++;
			wheelInterrupt[i]++;
			}
		if (switchOn==false && flagLow[i] == false)
			{

			flagLow[i] = true;
			}
		if (wheelInterrupt[i]>=wheelIdLimitation[i] && wheelIdLimitation[i]!=0 && wheelInterruptOn[i] == true)

		{
			lastWheelInterruptId=i;
			Serial.print("thre:");
			Serial.println(wheelInterrupt[i]);
			digitalWrite(softPinInterrupt,HIGH);
		}

	}
	}

#if defined(wheelEncoderDebugOn)

#endif
}