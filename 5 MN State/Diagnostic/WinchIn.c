#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     motorD,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     winchR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     winchL,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	while(true)
	{
		if (nNxtButtonPressed == 1)
  {
  	motor[winchR] = 100;
  }
  else if (nNxtButtonPressed == 2)
  {
   	motor[winchL] = 100;
  }
  else
  {
  	motor[winchR] = 0;
  	motor[winchL] = 0;
  }
}
}

