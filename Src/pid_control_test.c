#include "pid_control_test.h"

// PPM range : 2000 ~ 5000 ~ 8000
void PID_Control_Long_Initialize(LPID* dst, int select)
{
  switch(select)
  {
    case pos: //pos 
      	dst->errorSum = 0;
	dst->errorSumLimit = 500;

	dst->kP = 2000;
	dst->kI = 20;
	dst->kD = 0;

	dst->pastError = 0;
	dst->pastOutput = 0;
	dst->pastValue = 0;

	dst->underOfPoint = 100; 

	dst->outputLimit = 4000; // 300rpm -> 1800 degree/sec (300rpm)
      break;
      
    case vel: //vel
      	dst->errorSum = 0;
	dst->errorSumLimit = 500;

	dst->kP =  500;		
	dst->kI = 0;		
	dst->kD = 50;

	dst->pastError = 0;
	dst->pastOutput = 0;
	dst->pastValue = 0;

	dst->underOfPoint = 100000;

	dst->outputLimit = 30; // 50A-> pwm high
      break;
      
//    case cur: //cur
//      	dst->errorSum = 0;
//	dst->errorSumLimit = 500;
//
//	dst->kP = 50;		
//	dst->kI = 0;		
//	dst->kD = 0;
//
//	dst->pastError = 0;
//	dst->pastOutput = 0;
//	dst->pastValue = 0;
//
//	dst->underOfPoint = 10000; 
//
//	dst->outputLimit = 100; // 25V -> pwm high
//      break;
  }

}

void PID_Control(LPID* dst, double target, double input)
{
	dst->nowValue = input;
	dst->target = target; // 5000ÀÌ ¼Óµµ 0
        
	dst->nowError =  dst->target -dst->nowValue ;
	dst->errorSum += dst->nowError;
	dst->errorDiff = dst->nowError - dst->pastError;
	if(dst->errorSumLimit != 0)
	{
		if(dst->errorSum > dst->errorSumLimit)
			dst->errorSum = dst->errorSumLimit;
		else if(dst->errorSum < -dst->errorSumLimit)
			dst->errorSum = -dst->errorSumLimit;
	}
	dst->nowOutput = 
			(dst->kP * dst->nowError +
			dst->kI * dst->errorSum +
			dst->kD * dst->errorDiff);
        
	if(dst->underOfPoint == 0) return;	// Escape Error
	dst->nowOutput /= dst->underOfPoint;
	dst->pastError = dst->nowError;
	
	if(dst->outputLimit != 0)
	{
		if(dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
		else if(dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;
	}
}