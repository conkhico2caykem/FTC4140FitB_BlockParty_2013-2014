//include files so sensors/joysticks/other things work
//#include "hitechnic-gyro.h"
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
int _dirAC;
int IRdis;


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
	_dirAC = HTIRS2readACDir(HTIRS2);      //read IR seeker value
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
		_dirAC = HTIRS2readACDir(HTIRS2);
	}
		motor[rdrive] = 0;    //stops the robot
		motor[ldrive] = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

                             //uses gyro sensor to turn accurately

///////////////////////////////////////////////////////////////////////////////////////////////////

void gyroTurn(int target, int drivePowerR, int drivePowerL, bool turningRight, int waitTime, float timeout)
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
      if(heading > (target))
      {
        motor[rdrive] = -(drivePowerR);
        motor[ldrive] = (drivePowerL);
      }
      else
      {
        motor[rdrive] = 0;
        motor[ldrive] = 0;
        wait1Msec(waitTime);
        return;
      }
    }
    else
    {
      if(heading < (target))
      {
        motor[rdrive] = (drivePowerR);
        motor[ldrive] = -(drivePowerL);
      }
      else
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
	gyroTurn(degrees, 50, 50, false, 10, timeo);
}

void GyroRight(float degrees, float timeo)
{
	degrees = degrees * 0.93;
	ClearTimer(T1);
	gyroTurn(-degrees, 50, 50, true, 10, timeo);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////                             Straight, Gyro, W/ Encoder
////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void StraightGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)
{
  motor[rdrive] = 0;
  motor[ldrive] = 0;
  wait1Msec(waitTime);
  HTGYROstartCal(HTGYRO);
  float heading = 0.0;
  long currTime;
  long prevTime = nPgmTime;
  while(nMotorEncoder[rdrive] < encDis)
  {
    motor[rdrive] = power;
    motor[ldrive] = power;
    currTime = nPgmTime;
    heading += ((float)HTGYROreadRot(HTGYRO))*(currTime-prevTime)/1000;
    prevTime = currTime;
    if(heading < (target - buffer))  //if bot is rotated too far right
    {
      motor[rdrive] = ((power) + adjPower);
  		motor[ldrive] = ((power) - adjPower);
  	}
   	else if(heading > (target + buffer))   //if bot is too far left
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

void BackupGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)
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

void StraightIRGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)
{
  motor[rdrive] = 0;
  motor[ldrive] = 0;
  wait1Msec(waitTime);
  HTGYROstartCal(HTGYRO);
  float heading = 0.0;
  long currTime;
  long prevTime = nPgmTime;
  while(nMotorEncoder[rdrive] < encDis && _dirAC != 5)
  {
  	_dirAC = HTIRS2readACDir(HTIRS2);
    motor[rdrive] = power;
    motor[ldrive] = power;
    currTime = nPgmTime;
    heading += ((float)HTGYROreadRot(HTGYRO))*(currTime-prevTime)/1000;
    prevTime = currTime;
    if(heading < (target - buffer))  //if bot is rotated too far right
    {
      motor[rdrive] = ((power) + adjPower);
  		motor[ldrive] = ((power) - adjPower);
  	}
   	else if(heading > (target + buffer))   //if bot is too far left
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
  IRdis = nMotorEncoder[rdrive];
  motor[rdrive] = 0;
  motor[ldrive] = 0;
}

void BackupIRGyro(int power, int target, int buffer, int adjPower, int waitTime, int encDis)
{
  motor[rdrive] = 0;
  motor[ldrive] = 0;
  wait1Msec(waitTime);
  HTGYROstartCal(HTGYRO);
  float heading = 0.0;
  long currTime;
  long prevTime = nPgmTime;
  while(nMotorEncoder[rdrive] > encDis && _dirAC != 5)
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
  IRdis = nMotorEncoder[rdrive];
}

void straightIRG (int power, int adjPower, int encDis)
{
	nMotorEncoder[rdrive] = 0;
	StraightIRGyro (power, 0, 2, adjPower, 50, encDis);
}

void backupIRG (int power, int adjPower, int encDis)
{
	nMotorEncoder[rdrive] = 0;
	StraightIRGyro (-power, 0, 2, adjPower, 50, -encDis);
}

void backup (int power, int adjPower, int inches)
{
	nMotorEncoder[rdrive] = 0;
	int encDis = (inches * 142.8) + 190;
	BackupGyro(-power, 0, 2, adjPower, 50, -encDis);
}

void backupEnc (int power, int adjPower, int encDis)
{
	nMotorEncoder[rdrive] = 0;
	//int encDis = (inches * 142.8) + 190;
	BackupGyro(-power, 0, 2, adjPower, 50, -encDis);
}

void straight (int power, int adjPower, int inches)
{
	nMotorEncoder[rdrive] = 0;
	int encDis = (inches * 142.8) - 190;
	StraightGyro(power, 0, 2, adjPower, 50, encDis);
}

void straightEnc (int power, int adjPower, int encDis)
{
	nMotorEncoder[rdrive] = 0;
	//int encDis = (inches * 142.8) - 190;
	StraightGyro(power, 0, 2, adjPower, 50, encDis);
}
