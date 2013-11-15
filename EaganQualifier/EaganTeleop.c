#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     HTSMUX,         sensorI2CCustom)
#pragma config(Sensor, S4,     HTGYRO,         sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     rdrive,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     ldrive,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     kicker,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     flagspin,   tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     wench1,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     wench2,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    autoblock,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    irservo,              tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    wedgel,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    wedger,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    sabotage,             tServoStandard)
#pragma config(Servo,  srvo_S1_C4_6,    linney,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

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
    	rpower = (slowpower ) ;
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
  	motor[rdrive] = rpower;      //|
  	motor[ldrive] = rpower;      //| Set power to motors
}	// end TankDrive

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                       Wench/Lift                                                //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Wenches ()
{
	if (joy2Btn(5))
	{
		motor[wench1] = 30;    // spins spools towards front of bot
		motor[wench2] = 30;
	}
	else if (joy2Btn(7))
	{
		motor[wench1] = -30;   //spins spools towards back of bot
		motor[wench2] = -30;
	}
	else
	{
		motor[wench1] = 0;   //sets default to stop it
		motor[wench2] = 0;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                   Block Kicker/Wedge                                            //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

void BlockKicker()
{
	if(joy2Btn(6))
	{
		motor[kicker] = 100;
	}
	else if(joy2Btn(8))
	{
		motor[kicker] = -100;
	}
	else
	{
		motor[kicker] = 0;
	}
}

void kicklinney()
{
	if(joystick.joy2_TopHat == 0)
	{
		servo[linney] = 255;
	}
	else if(joystick.joy2_TopHat == 4)
	{
		servo[linney] = 0;
	}
	else
	{
		servo[linney] = 127;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      Flag Spinner                                               //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////
void FlagSpinner()
{
	if (joy2Btn(1))
	{
		motor[flagspin]= 25;
	}
	else
	{
		motor [flagspin] = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      IRservo                                                    //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                 //
//                                      AutoBlock                                                  //
//                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////
void AutonomousBlock()
{
	if (joy2Btn(3))
	{
		servo[autoblock] = 255;
	}
	else if(joy2Btn(2))
	{
		servo[autoblock] = 50;
	}
}

task main()
{
  initializeRobot();

  waitForStart();   // wait for start of tele-op phase

  while (true)
  {
  	getJoystickSettings(joystick);
  	TankDrive(joystick.joy1_TopHat, joystick.joy1_y1, joystick.joy1_y2);
  	Wenches();
  	BlockKicker();
  	kicklinney();
  	FlagSpinner();
  	AutonomousBlock();
  }
}