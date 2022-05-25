/*
use interrupt number 5


*/
//#define debugWheelControlOn  // uncomment to enter debug mode
#include <Arduino.h>
#include "WheelControl.h"
#define instantSpeedSize 8   // instant speed mesurment holes size
#define tcntWheelMin 65000  // 65535-tcntWheelMin determine the minimum interrupt freqency
//#define tcntWheelMax 65530  // 65535-tcntWheelMax determine the maximum interrupt freqency
#define tcntWheelMax 65500  // 65535-tcntWheelMax determine the maximum interrupt freqency
#define speedBias 0.95      // used to adjust GetTurnSpeed according to mesurment bias

unsigned int tcntWheel = tcntWheelMin;   // used to init timer overflow for instance 49911 for 1s cycle (tous les 8/100eme s
volatile float lastTurnWheelSpeed[4];
volatile float last2TurnWheelSpeed[4];
volatile long instantTurnSpeed[4][instantSpeedSize];
volatile unsigned int instantIdx[4];
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
volatile unsigned int prevTurnHolesCount[4];
unsigned int wheelIdIncoderHighValue[4];  
unsigned int wheelIdIncoderLowValue[4];
volatile int  wheelIdAnalogEncoderInput[4];
int _delayMiniBetweenHoles;
volatile unsigned int wheelIdLimitation[4];
uint8_t softPinInterrupt;
volatile boolean wheelInterruptOn[4];
volatile uint8_t lastWheelInterruptId;
uint8_t wheelIdEncoderHoles[4];
volatile unsigned int wheelPulseCount;
volatile boolean _controlOn=false;
volatile boolean _pulseOn=false;
volatile boolean _wheelControlOn[4];
WheelControl::WheelControl (
				uint8_t wheelId0EncoderHoles, int wheelId0IncoderHighValue, int wheelId0IncoderLowValue, int wheelId0AnalogEncoderInput,
				uint8_t wheelId1EncoderHoles, int wheelId1IncoderHighValue ,int wheelId1IncoderLowValue, int wheelId1AnalogEncoderInput, 
				uint8_t wheelId2EncoderHoles, int wheelId2IncoderHighValue ,int wheelId2IncoderLowValue, int wheelId2AnalogEncoderInput, 
				uint8_t wheelId3EncoderHoles, int wheelId3IncoderHighValue ,int wheelId3IncoderLowValue, int wheelId3AnalogEncoderInput, 
				uint8_t wheelPinInterrupt, float delayMiniBetweenHoles,float samplingRatio
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
			_delayMiniBetweenHoles=delayMiniBetweenHoles/2.5;
			tcntWheel = 65535 - delayMiniBetweenHoles/ samplingRatio *65535./10000; // for sampling at least 10 times between 2 holes 
//			tcntWheel = min(tcntWheel, tcntWheelMax);
//			tcntWheel = max(tcntWheel, tcntWheelMin);
			tcntWheel = min(tcntWheel, tcntWheelMax);
			}
void WheelControl::StartWheelControl(boolean wheelId0ControlOn, boolean wheelId0InterruptOn,unsigned int wheelId0Limitation,
						boolean wheelId1ControlOn,boolean wheelId1InterruptOn,unsigned int wheelId1Limitation,
						boolean wheelId2ControlOn,boolean wheelId2InterruptOn,unsigned int wheelId2Limitation,
						boolean wheelId3ControlOn,boolean wheelId3InterruptOn,unsigned int wheelId3Limitation
			)
			{
#if defined(debugWheelControlOn)
		Serial.println("StartWheelControl");
#endif
				_controlOn=true;
				_pulseOn=false;
				 if (softPinInterrupt!=0)
					{
					pinMode(softPinInterrupt,OUTPUT);
					digitalWrite(softPinInterrupt,HIGH);
					}
				if (wheelId0ControlOn)
				{
					_wheelControlOn[0]=true;
					wheelInterruptOn[0]=wheelId0InterruptOn;
					wheelIdLimitation[0]=wheelId0Limitation;
					lastTurnWheelSpeed[0] = 0;
					last2TurnWheelSpeed[0] = 0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[0][i] = 0;
					}
					instantIdx[0] = 0;
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
					microInt[0] = millis();
				}
				if (wheelId1ControlOn)
				{
					_wheelControlOn[1]=true;
					wheelInterruptOn[1]=wheelId1InterruptOn;
					wheelIdLimitation[1]=wheelId1Limitation;
					lastTurnWheelSpeed[1] = 0;
					last2TurnWheelSpeed[1] = 0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[1][i] = 0;
					}
					instantIdx[1] = 0;
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
					microInt[1] = millis();
				}
				if (wheelId2ControlOn)
				{
					_wheelControlOn[2]=true;
					wheelInterruptOn[2]=wheelId2InterruptOn;
					wheelIdLimitation[2]=wheelId2Limitation;
					lastTurnWheelSpeed[2] = 0;
					last2TurnWheelSpeed[2] = 0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[2][i] = 0;
					}
					instantIdx[2] = 0;
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
					microInt[2] = millis();
				}
				if (wheelId3ControlOn)
				{
					_wheelControlOn[3]=true;
					wheelInterruptOn[3]=wheelId3InterruptOn;
					wheelIdLimitation[3]=wheelId3Limitation;
					lastTurnWheelSpeed[3] = 0;
					last2TurnWheelSpeed[3] = 0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[3][i] = 0;
					}
					instantIdx[3] = 0;
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
					microInt[3] = millis();
				}
	/*
	set timer 5 interrupt parameters
	*/
				noInterrupts(); // disable all interrupts
				TCCR5A = 0;  // set entire TCCR5A register to 0
				TCCR5B = 0;  // set entire TCCR5B register to 0
				TCNT5 = tcntWheel; // sampling frequency = 64Khz/(65535-tcntWheel)
				TCCR5B |= ((1 << CS12) ); // 256 prescaler - frequency=16,000,000Hz/256 >> frequency=64KHz
				TIMSK5 |= (1 << TOIE5); // enable timer overflow interrupt
				interrupts(); // enable all interrupts
			}
void WheelControl::StartWheelPulse(unsigned int pulseLimitation)

			{
				_pulseOn=true;
				_controlOn=false;
				if (softPinInterrupt!=0)
				{
					pinMode(softPinInterrupt,OUTPUT);
					digitalWrite(softPinInterrupt,HIGH);
				}

					wheelIdLimitation[0]=pulseLimitation;
					wheelPulseCount = 0;
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

				noInterrupts(); // disable all interrupts
				TCCR5A = 0;  // set entire TCCR5A register to 0
				TCCR5B = 0;  // set entire TCCR5B register to 0
				TCNT5 = tcntWheel; // 
				TCCR5B |= ((1 << CS12) ); // 256 prescaler
				TIMSK5 |= (1 << TOIE5); // enable timer overflow interrupt
				interrupts(); // enable all interrupts
			}
void WheelControl::StopWheelControl(boolean wheelId0ControlOn,	boolean wheelId1ControlOn,boolean wheelId2ControlOn,boolean wheelId3ControlOn)
			{
#if defined(debugWheelControlOn)
		Serial.println("StopWheelControl");
#endif
				if (wheelId0ControlOn)
				{
					_wheelControlOn[0]=false;
					wheelInterruptOn[0]=false;
					wheelIdLimitation[0]=0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[0][i] = 0;
					}
					instantIdx[0] = 0;
				}
				if (wheelId1ControlOn)
				{
					_wheelControlOn[1]=false;
					wheelInterruptOn[1]=false;
					wheelIdLimitation[1]=0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[1][i] = 0;
					}
					instantIdx[1] = 0;
				}
				if (wheelId2ControlOn)
				{
					_wheelControlOn[2]=false;
					wheelInterruptOn[2]=false;
					wheelIdLimitation[2]=0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[2][i] = 0;
					}
					instantIdx[2] = 0;
				}
				if (wheelId3ControlOn)
				{
					_wheelControlOn[3]=false;
					wheelInterruptOn[3]=false;
					wheelIdLimitation[3]=0;
					for (int i = 0;i < instantSpeedSize;i++) {
						instantTurnSpeed[3][i] = 0;
					}
					instantIdx[3] = 0;
				}
				uint8_t countInt=0;
				for (int i=0;i<4;i++)
				{
					if (wheelInterruptOn[i]==true)
					{
						countInt++;
					}
				}
				if (countInt==0)  // stop timer
				{
				noInterrupts(); // disable all interrupts
				TCCR5A = 0;  // set entire TCCR5A register to 0
				TCCR5B = 0;  // set entire TCCR5B register to 0
				TCNT5 = 0; // 
				TCCR5B |= ((0 << CS10) ); // 
				TCCR5B |= ((0 << CS11) ); //
				TCCR5B |= ((0 << CS12) ); // 256 prescaler
				TIMSK5 |= (0 << TOIE5); // enable timer overflow interrupt
				interrupts(); // enable all interrupts
				_controlOn=false;
				}
			}
void WheelControl::IncreaseThreshold(uint8_t wheelId, unsigned int pulseIncrease )

			{
					wheelIdLimitation[wheelId]=wheelIdLimitation[wheelId]+pulseIncrease;
	#if defined(debugWheelControlOn)
					Serial.print("increase limit:");
					Serial.print(wheelId);
					Serial.print("-");					
					Serial.println(wheelIdLimitation[wheelId]);
	#endif
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
		if (lastTurnWheelSpeed[wheelId] == 0) {
			return GetInstantTurnSpeed(wheelId);
		}
		else {
			return lastTurnWheelSpeed[wheelId];
		}
	}
float WheelControl::Get2LastTurnSpeed(uint8_t wheelId)
		{
		if (last2TurnWheelSpeed[wheelId] == 0) {
			return GetLastTurnSpeed(wheelId);
		}
		else {
			return last2TurnWheelSpeed[wheelId];
		}
	}
float WheelControl::GetTurnSpeed(uint8_t wheelId)
{
	float speed = ((GetLastTurnSpeed(wheelId) + 2 * Get2LastTurnSpeed(wheelId) + GetInstantTurnSpeed(wheelId)) / 4) * speedBias;
	//if (prevTurnHolesCount[wheelId] == GetCurrentHolesCount(wheelId)) {
	//	speed = 0;
	//}
	prevTurnHolesCount[wheelId] = GetCurrentHolesCount(wheelId);
	return speed;
}
float WheelControl::GetInstantTurnSpeed(uint8_t wheelId)
{
	float speed = 0;
	unsigned long deltaTime = 0;
	if (wheelInterrupt[wheelId] > instantSpeedSize) { // instantTurnSpeed[wheelId] is completed
	//	deltaTime=(instantTurnSpeed[wheelId][(instantIdx[wheelId] - 1) % instantSpeedSize] - instantTurnSpeed[wheelId][instantIdx[wheelId]]);
		deltaTime = (millis() - instantTurnSpeed[wheelId][instantIdx[wheelId]]);
		deltaTime = deltaTime / (instantSpeedSize - 1);
	}
	else {
		deltaTime = (millis() - instantTurnSpeed[wheelId][wheelInterrupt[wheelId]]);
		deltaTime = deltaTime / (wheelInterrupt[wheelId]);
	}

	if (deltaTime != 0) {
		speed = (1000. / (deltaTime * wheelIdEncoderHoles[wheelId]));
	}
/*	Serial.print("-");
	Serial.println(speed);
	*/
	return speed;
}

unsigned int WheelControl::GetWheelThreshold(uint8_t wheelId)
	{
	return wheelIdLimitation[wheelId];
	}
unsigned int WheelControl::GetWheeLowValue(uint8_t wheelId)
{
	return wheelIdIncoderLowValue[wheelId];
}
unsigned int WheelControl::GetWheeHighValue(uint8_t wheelId)
{
	return wheelIdIncoderHighValue[wheelId];
}
uint8_t WheelControl::GetLastWheelInterruptId()
	{
	return(lastWheelInterruptId);
	}
void WheelControl::ClearThreshold(uint8_t wheelId)
{
	wheelIdLimitation[wheelId]=0;
}
ISR(TIMER5_OVF_vect)        // timer interrupt used to regurarly check rotation
{
	TCNT5 = tcntWheel;            // preload timer to adjust duration
#if defined(debugWheelControlOn)

#endif
	if (_controlOn==true)
	{
			for (int i=0;i<4;i++)
			{
				if (_wheelControlOn[i])
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
					if (millis()-microInt[i]> _delayMiniBetweenHoles)    //  delay not big enough do nothing to avoid misreading
					{
						volatile boolean switchOn=false;  
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
						 if (level < wheelIdIncoderLowValue[i] && startHigh[i]==true)     // started high and low now
						 {
							 switchOn=false;
						 }
						if (switchOn==true && flagLow[i] == true)  // one new hole detected
						{
#if defined(debugWheelControlOn)
	Serial.print("~");
	Serial.println(i);
#endif
							if(_controlOn==true)
							{
								microInt[i]=millis();
								flagLow[i] = false;

								if (wheelInterrupt[i]%wheelIdEncoderHoles[i]==(wheelIdEncoderHoles[i]-1))  // get time for the last occurence
								{
									lastTurnWheelSpeed[i] = float(1000. / (millis() - timerInt[i]));
								}
								if (wheelInterrupt[i]%(2*wheelIdEncoderHoles[i])==(2*wheelIdEncoderHoles[i]-1))  // get time for the last occurence
								{
									last2TurnWheelSpeed[i] = float(2000. / (millis() - timer2Int[i]));
								}
								if (wheelInterrupt[i] % wheelIdEncoderHoles[i] == 0)  // get time for the first occurence
								{
									timerInt[i] = millis();
								}
								if (wheelInterrupt[i] % (2 * wheelIdEncoderHoles[i]) == 0)  // get time for the first occurence
								{
									timer2Int[i] = millis();
								}							
								instantTurnSpeed[i][instantIdx[i]] = microInt[i];
								instantIdx[i] = (instantIdx[i]+1) % instantSpeedSize;
								wheelInterrupt[i]++;
							}
						}
						if (switchOn==false && flagLow[i] == false)
							{
							flagLow[i] = true;
							}
						if (wheelInterrupt[i]>=wheelIdLimitation[i] && wheelIdLimitation[i]!=0 && wheelInterruptOn[i] == true)
						{
							lastWheelInterruptId=i;
				#if defined(debugWheelControlOn)
							Serial.print("threshold reached:");
							Serial.print(i);
							Serial.print("-");
							Serial.println(wheelInterrupt[i]);
				#endif
							digitalWrite(softPinInterrupt,LOW);
							wheelInterruptOn[i]=false;     // no more interrupt needed
						}
					}
				}
			}
		}
	
	if (_pulseOn==true)
	{
			wheelPulseCount++;
			if (wheelPulseCount>=wheelIdLimitation[0])
			{
				lastWheelInterruptId=5;
				wheelIdLimitation[0] = 0;
				noInterrupts(); // disable all interrupts
				TCCR5A = 0;  // set entire TCCR5A register to 0
				TCCR5B = 0;  // set entire TCCR5B register to 0
				TCNT5 = 0; // 
				TCCR5B |= ((0 << CS10) ); // 
				TCCR5B |= ((0 << CS11) ); //
				TCCR5B |= ((0 << CS12) ); // 256 prescaler
				TIMSK5 |= (0 << TOIE5); // enable timer overflow interrupt
				interrupts(); // enable all interrupts
				_pulseOn=false;
#if defined(debugWheelControlOn)
				Serial.print("Pulse reached:");
				Serial.println(wheelPulseCount);
				delay(100);
	#endif				
				digitalWrite(softPinInterrupt,LOW);
			}
		}
}