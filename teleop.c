#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTGYRO,         sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     ldrive,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rdrive,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flagspinner,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     blockkicker,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     wenchl,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     wenchr,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    autoblock,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    irservo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    wedgel,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    wedger,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    sabotage,             tServoStandard)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
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

//variables
float rpower;
float lpower;
int deadzone = 10;   //for joysticks
int slowpower = 60.0;  //used on tophat for minor adjustments

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

void TankDrive(int topHat, int y1, int y2)
{
	switch (topHat) // TopHat butons have priority over joysticks
	{
		case 0: // Top Hat Up (Button 0) = Forward Slow
    	rpower = (slowpower) ;
    	lpower = slowpower;
    	//motor[wrist] = 7;
    	break;
  	case 4: // Top Hat Down (Button 4) = Backward Slow
    	rpower = -(slowpower) + 10;
    	lpower = -slowpower + 10;
      //motor[wrist] = -7;
    	break;
  	case 6: // Top Hat Left (Button 6) = Left Spin Slow
    	rpower = (slowpower + 20) - 23;
    	lpower = -(slowpower + 20);
    	break;
  	case 2: // Top Hat Right (Button 2) = Right Spin Slow
    	rpower = -((slowpower + 20) - 23);
    	lpower = (slowpower + 20);
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
  	motor[rdrive] = rpower;      //|Set power to motors
  	motor[ldrive] = lpower;      //|
}	// end TankDrive

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                       Wench/Lift                                                //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                   Block Kicker/Wedge                                            //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      Flag Spinner                                              //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      IRservo                                                    //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////



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
	  ///////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////
	  ////                                                   ////
	  ////      Add your robot specific tele-op code here.   ////
	  ////                                                   ////
	  ///////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////

    // Insert code to have servos and motors respond to joystick and button values.

    // Look in the ROBOTC samples folder for programs that may be similar to what you want to perform.
    // You may be able to find "snippets" of code that are similar to the functions that you want to
    // perform.
  }
}
