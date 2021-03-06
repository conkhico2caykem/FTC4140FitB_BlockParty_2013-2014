#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S2, HTServo,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     ldrive,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rdrive,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     winchr,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     winchl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     flagspin,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
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
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Tele-Operation Mode Code Template
//
// This file contains a template for simplified creation of an tele-op program for an FTC
// competition.
//
// You need to customize two functions with code unique to your specific robot.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

//variables for the drivetrain
float rpower;
float lpower;
int deadzone = 10;   //for joysticks
int slowpower = 30.0;  //used on tophat for minor adjustments

//varibles for the scoop
int offset = -6;
int RscoopTarget;
int LscoopTarget;

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of tele-op mode, you may want to perform some initialization on your robot
// and the variables within your program.
//
// In most cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot()
{
  // Place code here to sinitialize servos to starting positions.
  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                        Tank Drive                                               //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

void drivetrain(int topHat, int y1, int y2)
{
	switch (topHat) // TopHat butons have priority over joysticks
	{
		case 0: // Top Hat Up (Button 0) = Forward Slow
    	rpower = (slowpower ) ;
    	lpower = slowpower;
    	//motor[wrist] = 7;
    	break;
  	case 4: // Top Hat Down (Button 4) = Backward Slow
    	rpower = -(slowpower);
    	lpower = -slowpower;
      //motor[wrist] = -7;
    	break;
  	case 6: // Top Hat Left (Button 6) = Left Spin Slow
    	rpower = (slowpower);
    	lpower = -(slowpower);
    	break;
  	case 2: // Top Hat Right (Button 2) = Right Spin Slow
    	rpower = -((slowpower));
    	lpower = (slowpower);
    	break;
  	default:
			rpower = (y2 / 128.0) * 100; // rpower Proportional to right analog stick's Y-axis value
  		lpower = (y1 / 128.0) * 100; // lpower proportional to left analog stick's Y-axis value
  	if (abs(lpower) < deadzone)
  	{
  		lpower = 0;  // Left side deadzone
	  }
  	if (abs(rpower) < deadzone)
  	{
	    rpower = 0;  // Right side deadzone
	  }
	}
  	motor[rdrive] = rpower;      //|
  	motor[ldrive] = lpower;      //| Set power to motors
}	// end TankDrive

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                       Winch/Lift                                                //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

void winch(int y1, int y2)
{
	rpower = (y2 / 128.0) * 100; // rpower Proportional to right analog stick's Y-axis value
  lpower = (y1 / 128.0) * 100; // lpower proportional to left analog stick's Y-axis value
  if (abs(lpower) < deadzone)
  {
  	lpower = 0;  // Left side deadzone
	}
  	if (abs(rpower) < deadzone)
  {
	  rpower = 0;  // Right side deadzone
	}
	motor[winchl] = lpower; //sets motor power equal to the valu of the joystick
	motor[winchr] = rpower; //               ||
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                       Flag Spinner                                              //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

void flag()
{
	int flagPower = 20;
	if (joy2Btn(5))   //if top L trigger is pressed spins flag up
	{
		motor[flagspin]= flagPower; //sets motor to flagPower
		if (flagPower <= 100) //limits motorPower to 100
		{
			flagPower = flagPower + 1; //gradually increases flagPower so we dont rip the handle off
			wait1Msec(25);
		}
	}
	else if (joy2Btn(7)) //if bottom L trigger flag down
	{
		motor[flagspin] = -60; //spins flag down slowerly
	}
	else
	{
		motor [flagspin] = 0; //if no buttons are pressed the motor stops
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      Scoop                                                      //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

//L servo 0 = up
//L servo 255 = down
//R servo 255 = up
//R servo 0 = down

void scoop()
{
	if (joy1Btn(6)) //erin's top right trigger
	{
		RscoopTarget = 20; //brigns the scoop up
	}
	else if (joy1Btn(8)) //erin's bottom right trigger
	{
		RscoopTarget = 255; //brings the scoop down
	}
	else if (joy1Btn(7) && RscoopTarget < 255) //erin's top left trigger
	{
		RscoopTarget = RscoopTarget + 1; // gradually brings scoop up
		wait1Msec(5);
	}
	else if (joy1Btn(5) && RscoopTarget > 20) //erin's bottom left trigger
	{
		RscoopTarget = RscoopTarget - 1; //gradually brings scoop down
		wait1Msec(5);
	}
	else if (joy2Btn(4)) // crystal's top left trigger
	{
		RscoopTarget = 30; // bring scoop up (hot button)
	}
	else if (joy2Btn(2)) //crystal's bottom left trigger
	{
		RscoopTarget = 225; //brings scoop down (hot button)
	}
	else if (joy2Btn(3))
	{
		RscoopTarget = 78;
	}
	else if (joy2Btn(10))
	{
		servo[scoopflap] = 255;
	}
	else if (joy2Btn(1))
	{
		servo[scoopflap] = 0;
	}
	LscoopTarget = 	255 - RscoopTarget + offset;
	servo[scoopR] = RscoopTarget;
	servo[scoopL] = LscoopTarget;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                         Main Task
//
// The following is the main code for the tele-op robot operation. Customize as appropriate for
// your specific robot.
//
// Game controller / joystick information is sent periodically (about every 50 milliseconds) from
// the FMS (Field Management System) to the robot. Most tele-op programs will follow the following
// logic:
//   1. Loop forever repeating the following actions:
//   2. Get the latest game controller / joystick settings that have been received from the PC.
//   3. Perform appropriate actions based on the joystick + buttons settings. This is usually a
//      simple action:
//      *  Joystick values are usually directly translated into power levels for a motor or
//         position of a servo.
//      *  Buttons are usually used to start/stop a motor or cause a servo to move to a specific
//         position.
//   4. Repeat the loop.
//
// Your program needs to continuously loop because you need to continuously respond to changes in
// the game controller settings.
//
// At the end of the tele-op period, the FMS will autonmatically abort (stop) execution of the program.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

task main()
{
  initializeRobot();

  waitForStart();   // wait for start of tele-op phase

  while (true)
  {
  	getJoystickSettings(joystick);
  	drivetrain(joystick.joy1_TopHat, joystick.joy1_y1, joystick.joy1_y2);
  	winch(joystick.joy2_y1, joystick.joy2_y2);
  	flag();
  	scoop();
  }
}
