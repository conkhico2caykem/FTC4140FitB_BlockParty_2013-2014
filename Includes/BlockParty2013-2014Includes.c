#include "hitechnic-gyro.h"
#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "hitechnic-touchmux.h"   //include file for the touch sensor mux
#include "hitechnic-gyro.h"   //include for the gyro
#include "hitechnic-sensormux.h"
#include "lego-ultrasound.h"    //for sonar sensor
#include "hitechnic-irseeker-v2.h"    //for IR seeker sensor
#include "lego-touch.h"    //for touch sensor
#include "hitechnic-eopd.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             moves for distance using encoders
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
int encval;

void MFD(int inches, float timeout)
{
	nMotorEncoder[rdrive] = 0;
	encval = (inches * 142.8) - 190;   //convert inches to enc clicks
	ClearTimer(T1);

	while((nMotorEncoder[rdrive] < encval)) //monitors encoder value
	{
		motor[ldrive] = 100;    //moves robot forward
		motor[rdrive] = 100;
		if(time1[T1] > timeout)
		{
			StopAllTasks();
		}
	}
	motor[ldrive] = 0;    //moves robot stop
	motor[rdrive] = 0;
}

void MBD(int inches, float timeout)
{
	nMotorEncoder[rdrive] = 0;
	encval = (inches * 142.8) - 190;   //convert inches to enc clicks
	ClearTimer(T1);
	while(nMotorEncoder[rdrive] > (-encval)) //monitors encoder value
	{
		motor[rdrive] = -100;    //moves robot forward
		motor[ldrive] = -100;
		if(time1[T1] >= timeout)
		{
			StopAllTasks();
		}
	}
		motor[ldrive] = 0;    //moves robot stop
		motor[rdrive] = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             uses gyro sensor to turn accurately
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void gyroTurn(int target, int buffer, int drivePowerR, int drivePowerL, bool turningRight, int waitTime, float timeout)
{
  motor[ldrive] = 0;
  motor[rdrive] = 0;
  wait1Msec(waitTime);
  HTGYROstartCal(HTGYRO);

  float heading = 0.0;
  long currTime;
  long prevTime = nPgmTime;

  while(true)
  {
    currTime = nPgmTime;
    heading += ((float)HTGYROreadRot(HTGYRO))*(currTime-prevTime)/1000;
    prevTime = currTime;
    if(turningRight)
    {
      if(heading < (target - buffer))
      {
        motor[rdrive] = -(drivePowerR);
        motor[ldrive] = (drivePowerL);
      }
      else if(heading > (target + buffer))
      {
        motor[rdrive] = (drivePowerR);
        motor[ldrive] = -(drivePowerL);
      }
      else if(heading < (target + buffer) && heading > (target - buffer))
      {
        motor[rdrive] = 0;
        motor[ldrive] = 0;
        wait1Msec(waitTime);
        return;
      }
    }
    else if(turningRight == false)
    {
      if(heading < (target - buffer))
      {
        motor[rdrive] = (drivePowerR);
        motor[ldrive] = -(drivePowerL);
      }
      else if(heading > (target + buffer))
      {
        motor[rdrive] = -(drivePowerR);
        motor[ldrive] = (drivePowerL);
      }
      else if(heading < (target + buffer) && heading > (target - buffer))
      {
        motor[ldrive] = 0;
        motor[rdrive] = 0;
        wait1Msec(waitTime);
        return;
      }
    }
    if(time1[T1] > timeout)
    {
    	StopAllTasks();
    }
  }
}

void GyroLeft(float degrees, float timeo)
{
	degrees = degrees * 0.93;
	ClearTimer(T1);
	gyroTurn(degrees, 3, 100, 77, false, 10, timeo);
}

void GyroRight(float degrees, float timeo)
{
	degrees = degrees * 0.93;
	ClearTimer(T1);
	gyroTurn(-degrees, 3, 100, 77, false, 10, timeo);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////                             Straight, Gyro, W/ Encoder
////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void StraightGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)  //encDis is a negative number when going forward for both motors
{
  motor[ldrive] = 0;
  motor[rdrive] = 0;
  wait1Msec(waitTime);
  HTGYROstartCal(HTGYRO);
  float heading = 0.0;
  long currTime;
  long prevTime = nPgmTime;
  while(nMotorEncoder[rdrive] > encDis)
  {
    motor[rdrive] = power;
    motor[ldrive] = power;
    currTime = nPgmTime;
    heading += ((float)HTGYROreadRot(HTGYRO))*(currTime-prevTime)/1000;
    prevTime = currTime;
    if(heading < (target - buffer))
    {
      motor[rdrive] = ((power) + adjPower);
  		motor[ldrive] = ((power) - adjPower);
  	}
   	else if(heading > (target + buffer))
    {
    	motor[rdrive] = ((power) - adjPower);
  		motor[ldrive] = ((power) + adjPower);
   	}
   	else if(heading < (target + buffer) && heading > (target - buffer))
   	{
    	motor[ldrive] = power;
  		motor[rdrive] = power;
   	}
  }
  motor[ldrive] = 0;
  motor[rdrive] = 0;
}

void straight (int power, int adjPower, int encDis)
{
	StraightGyro(power, 0, 3, adjPower, 100, encDis);
}
