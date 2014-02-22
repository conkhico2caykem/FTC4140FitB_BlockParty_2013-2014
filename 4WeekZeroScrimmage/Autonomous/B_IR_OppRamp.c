#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S3,     HTGYRO,         sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     HTIRS2,         sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     ldrive,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rdrive,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     winchr,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     winchl,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_1,     flagspin,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     shooter,       tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C4_2,     motorK,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    scoopR,               tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    scoopL,               tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    turret,               tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    scoopflap,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_5,    autoblock,            tServoStandard)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Autonomous Mode Code Template
//
// This file contains a template for simplified creation of an autonomous program for an TETRIX robot
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.
#include "BlockPartyIncludes.c"



/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of autonomous mode, you may want to perform some initialization on your robot.
// Things that might be performed during initialization include:
//   1. Move motors and servos to a preset position.
//   2. Some sensor types take a short while to reach stable values during which time it is best that
//      robot is not moving. For example, gyro sensor needs a few seconds to obtain the background
//      "bias" value.
//
// In many cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

int _dirAC;

void initializeRobot()
{
	servo[scoopL] = 0;
	servo[scoopR] = 255;
	servo[autoblock] = 0;  //make autoarm legal
  // Place code here to sinitialize servos to starting positions.
  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.

  return;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                         Main Task
//
// The following is the main code for the autonomous robot operation. Customize as appropriate for
// your specific robot.
//
// The types of things you might do during the autonomous phase (for the 2008-9 FTC competition)
// are:
//
//   1. Have the robot follow a line on the game field until it reaches one of the puck storage
//      areas.
//   2. Load pucks into the robot from the storage bin.
//   3. Stop the robot and wait for autonomous phase to end.
//
// This simple template does nothing except play a periodic tone every few seconds.
//
// At the end of the autonomous period, the FMS will autonmatically abort (stop) execution of the program.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

task main()
{
  initializeRobot();

  waitForStart(); // Wait for the beginning of autonomous phase.

  servo[scoopL] = 200;
	servo[scoopR] = 55;

	MBD(20, 100, 1000000);
	wait10Msec(10);
	_dirAC = HTIRS2readACDir(HTIRS2);      //read IR seeker value
	if(_dirAC > 5)    //check zone 1
	{
		//insert code to dump block
		motor[rdrive] = 0;
		motor[ldrive] = 0;
		servo[autoblock] = 250;
		wait10Msec(100); //for testing only, remove when block dump is added  THIS IS FOR ALL WAIT(3000000)!!!!
		servo[autoblock] = 5;
		wait10Msec(70);
		servo[scoopL] = 255;
		servo[scoopR] = 0;
		MBD(43, 100, 100000);  //Straight to end of ramp (distance changes depending on what basket your at)
		wait10Msec(10);
		GyroLeft(39, 100000000); //turn 45 degrees to drive parralel to ramp
		wait10Msec(10);
		MBD(16, 100, 1000000); //drives forward a little to turn
		wait10Msec(10);
		GyroLeft(35, 10000000); //turn to drive towards white line
		wait10Msec(10);
		MBD(50, 100, 1000000); //drives forward to first white line  (change for second white line)
		wait10Msec(10);
	  GyroLeft(88, 100000);//turn to drive on ramp
	  wait10Msec(10);
	  MBD(38, 100, 1000000);//drives on ramp an inch past the midpoint
	}
	else
	{
		MBD(10, 100, 100000);
		wait10Msec(100);
	  _dirAC = HTIRS2readACDir(HTIRS2);      //read IR seeker value
		if(_dirAC > 5)  //check zone 2
		{
				//insert code to dump block
			motor[rdrive] = 0;
			motor[ldrive] = 0;
			servo[autoblock] = 250;
			wait10Msec(100); //for testing only, remove when block dump is added  THIS IS FOR ALL WAIT(3000000)!!!!
			servo[autoblock] = 5;
			wait10Msec(70);
			servo[scoopL] = 255;
			servo[scoopR] = 0;
			MBD(33, 100, 100000);  //Straight to end of ramp (distance changes depending on what basket your at)
			wait10Msec(10);
			GyroLeft(39, 100000000); //turn 45 degrees to drive parralel to ramp
			wait10Msec(10);
			MBD(16, 100, 1000000); //drives forward a little to turn
			wait10Msec(10);
			GyroLeft(35, 10000000); //turn to drive towards white line
			wait10Msec(10);
			MBD(50, 100, 1000000); //drives forward to first white line  (change for second white line)
			wait10Msec(10);
	  	GyroLeft(88, 100000);//turn to drive on ramp
	  	wait10Msec(10);
	  	MBD(38, 100, 1000000);//drives on ramp an inch past the midpoint
		}
		else
		{
			MBD(14, 100, 1000000);
			wait10Msec(100);
			 _dirAC = HTIRS2readACDir(HTIRS2);      //read IR seeker value
			if(_dirAC > 4)  //check zone 3
			{
				MBD(3, 100, 100000);
				motor[rdrive] = 0;
				motor[ldrive] = 0;
				servo[autoblock] = 250;
				wait10Msec(100); //for testing only, remove when block dump is added  THIS IS FOR ALL WAIT(3000000)!!!!
				servo[autoblock] = 5;
				wait10Msec(70);
				servo[scoopL] = 255;
				servo[scoopR] = 0;
				MBD(16, 100, 100000);  //Straight to end of ramp (distance changes depending on what basket your at)
				wait10Msec(10);
				GyroLeft(39, 100000000); //turn 45 degrees to drive parralel to ramp
				wait10Msec(10);
				MBD(16, 100, 1000000); //drives forward a little to turn
				wait10Msec(10);
				GyroLeft(35, 10000000); //turn to drive towards white line
				wait10Msec(10);
				MBD(50, 100, 1000000); //drives forward to first white line  (change for second white line)
				wait10Msec(10);
	  		GyroLeft(88, 100000);//turn to drive on ramp
	  		wait10Msec(10);
	  		MBD(38, 100, 1000000);//drives on ramp an inch past the midpoint
			}
			else  //just dump in zone 4
			{
				MBD(13, 100, 10000000);
				motor[rdrive] = 0;
				motor[ldrive] = 0;
				servo[autoblock] = 250;
				wait10Msec(100); //for testing only, remove when block dump is added  THIS IS FOR ALL WAIT(3000000)!!!!
				servo[autoblock] = 5;
				wait10Msec(70);
				servo[scoopL] = 255;
				servo[scoopR] = 0;
				MBD(3, 100, 10000000);  //Straight to end of ramp (distance changes depending on what basket your at)
				wait10Msec(10);
				GyroLeft(39, 100000000); //turn 45 degrees to drive parralel to ramp
				wait10Msec(10);
				MBD(16, 100, 1000000); //drives forward a little to turn
				wait10Msec(10);
				GyroLeft(35, 10000000); //turn to drive towards white line
				wait10Msec(10);
				MBD(50, 100, 1000000); //drives forward to first white line  (change for second white line)
				wait10Msec(10);
	  		GyroLeft(88, 100000);//turn to drive on ramp
	  		wait10Msec(10);
	  		MBD(38, 100, 1000000);//drives on ramp an inch past the midpoint
			}
		}
	}
}
