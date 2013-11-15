//include files so sensors/joysticks/other things work
#include "hitechnic-gyro.h"
#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "hitechnic-touchmux.h"   //include file for the touch sensor mux
#include "hitechnic-gyro.h"   //include for the gyro
#include "hitechnic-sensormux.h"
#include "lego-ultrasound.h"    //for sonar sensor
#include "hitechnic-irseeker-v2.h"    //for IR seeker sensor
#include "lego-touch.h"    //for touch sensor
#include "hitechnic-eopd.h"

//variables
int encval;


//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             moves for distance using encoders
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void MFD(int inches, int power, float timeout)  //moves forwards for a set distance based on encoders
{
	nMotorEncoder[rdrive] = 0; //reset encoder
	encval = (inches * 142.8) - 190;   //convert inches to enc clicks
	ClearTimer(T1); //cears timer

	while((nMotorEncoder[rdrive] < encval)) //monitors encoder value
	{
		motor[rdrive] = power;    //moves robot forward
		motor[ldrive] = power;
		if(time1[T1] > timeout)
		{
			StopAllTasks();
		}
	}
	motor[rdrive] = 0;    //moves robot stop
	motor[ldrive] = 0;
}

void MBD(int inches, int power, float timeout)
{
	nMotorEncoder[rdrive] = 0; //resets encoders
	encval = (inches * 142.8) - 190;   //convert inches to enc clicks
	ClearTimer(T1); //resets timer for the timeout
	while(nMotorEncoder[rdrive] > (-encval)) //monitors encoder value
	{
		motor[rdrive] = -power;    //moves robot backwards
		motor[ldrive] = -power;
		if(time1[T1] >= timeout)  //checks to see if we have been running longer than we should have
		{
			StopAllTasks();  //if so, stops everything/exits out of program
		}
	}
		motor[rdrive] = 0;    //stops the robot
		motor[ldrive] = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             uses gyro sensor to turn accurately
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void gyroTurn(int target, /*int buffer,*/ int drivePowerR, int drivePowerL, bool turningRight, int waitTime, float timeout)
{
  motor[rdrive] = 0;
  motor[ldrive] = 0;
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
      if(heading > (target /*- buffer*/))
      {
        motor[rdrive] = -(drivePowerR);
        motor[ldrive] = (drivePowerL);
      }
      //else if(heading > (target + buffer))
      //{
      //  motor[rdrive] = (drivePowerR);
      //  motor[ldrive] = -(drivePowerL);
      //}
      else /*if(heading < (target + buffer) && heading > (target - buffer))*/
      {
        motor[rdrive] = 0;
        motor[ldrive] = 0;
        wait1Msec(waitTime);
        return;
      }
    }
    else
    {
      if(heading < (target/* - buffer*/))
      {
        motor[rdrive] = (drivePowerR);
        motor[ldrive] = -(drivePowerL);
      }
      //else if(heading > (target + buffer))
      //{
      //  motor[rdrive] = -(drivePowerR);
      //  motor[ldrive] = (drivePowerL);
      //}
      else /*if(heading < (target + buffer) && heading > (target - buffer))*/
      {
        motor[rdrive] = 0;
        motor[ldrive] = 0;
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
	gyroTurn(degrees, /*3,*/ 100, 77, false, 10, timeo);
}

void GyroRight(float degrees, float timeo)
{
	degrees = degrees * 0.93;
	ClearTimer(T1);
	gyroTurn(-degrees,/* 3,*/ 100, 77, true, 10, timeo);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////                             Straight, Gyro, W/ Encoder
////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void StraightGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)  //encDis is a negative number when going forward for both motors
{
  motor[rdrive] = 0;
  motor[ldrive] = 0;
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
    	motor[rdrive] = power;
  		motor[ldrive] = power;
   	}
  }
  motor[rdrive] = 0;
  motor[ldrive] = 0;
}

void straight (int power, int adjPower, int encDis)
{
	StraightGyro(power, 0, 3, adjPower, 100, encDis);
}
