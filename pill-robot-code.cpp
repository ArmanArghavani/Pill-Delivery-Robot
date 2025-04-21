// extra libraries
#include "PC_FileIO.c";

// global constants
const int MAX = 10;
const int NUMTURN = 20;

// configure sensors function
void configAllSensors()
{
	SensorType[S1] = sensorEV3_Ultrasonic;
	wait1Msec(20);

	SensorType[S2] = sensorEV3_IRSensor;
	wait1Msec(100);

	SensorMode[S2] = modeEV3IR_Seeker;
	wait1Msec(1000);

	SensorMode[S2] = modeEV3IR_Calibration;
	wait1Msec(3000);

	SensorType[S3] = sensorEV3_Gyro;
	wait1Msec(50);

	SensorMode[S3] = modeEV3Gyro_Calibration;
	wait1Msec(100);

	SensorMode[S3] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);

	SensorType[S4] = sensorEV3_Touch;
	wait1Msec(20);

}

// read pill schedule into arrays
void readSchedule(int * scheduleHours, int * scheduleMinutes, int * pillCounts, int  & scheduleSize)
{
	TFileHandle fin;
	bool fileOkay = openReadPC(fin, "pill_schedule.txt");

	int hour = 0, minute = 0, pills = 0;

	if(!fileOkay)
	{
		writeDebugStream("Error opening file\n");
		return;
	}

	if(fileOkay)
		displayString(3, "file ok");

	while(scheduleSize < 3 && readIntPC(fin, hour) && readIntPC(fin, minute) && readIntPC(fin, pills))
	{
		scheduleHours[scheduleSize] = hour;
		scheduleMinutes[scheduleSize] = minute;
		pillCounts[scheduleSize] = pills;

		displayString(3+scheduleSize, "%d,%d,%d", hour, minute, pills);

		(scheduleSize)++;
	}
	closeFilePC(fin);
}

// dispense pills
void dispensePills(int * pillCounts, int & scheduleIndex)
{
	nMotorEncoder[motorB]= 0;
	motor[motorB] = -15;

	while(abs(nMotorEncoder[motorB]) < 360*pillCounts[scheduleIndex])
	{}

	motor[motorB] = 0;
	scheduleIndex++;
}

// get the current time in hours and minutes
void getCurrentTime(int & currentHour, int & currentMinute)
{
	int totalMinutes = time1[T1] / 60000; // converts milliseconds to minutes
	currentHour = (totalMinutes / 60) % 24;
	currentMinute = totalMinutes % 60;
}

// check if its time to dispense pills
bool checkTime(int * scheduleHours, int * scheduleMinutes, int & scheduleIndex, int & scheduleSize)
{
	int currentHour = 0, currentMinute = 0;
	getCurrentTime(currentHour, currentMinute);

	if(scheduleIndex < scheduleSize)
		if(scheduleHours[scheduleIndex] == currentHour && scheduleMinutes[scheduleIndex] == currentMinute)
			return true;

	return false;
}

// stop motors
void stopD()
{
	motor[motorA] = motor[motorD] = 0;
}

// slow down
void slowD(float Kp)
{
    motor[motorA] = motor[motorD] = Kp*(getUSDistance(S1) - 25);
}

// turns left
void turnLeft(float angle, float Kp)
{
    resetGyro(S3);
    motor[motorA] = Kp*(angle - abs(getGyroDegrees(S3)));
    motor[motorD] = -Kp*(angle - abs(getGyroDegrees(S3)));
    while(abs(getGyroDegrees(S3)) < angle){}
}

// turns right
void turnRight(float angle, float Kp)
{
    resetGyro(S3);
    motor[motorA] = -Kp*(angle - abs(getGyroDegrees(S3)));
    motor[motorD] = Kp*(angle - abs(getGyroDegrees(S3)));
    while(abs(getGyroDegrees(S3)) < angle){}
    stopD();
}

// rotate robot
void rotateRobot(int angle, int motorPower)
{
    motorPower = abs(motorPower);
    if (angle < 0)
    {
        motor[motorA] = motorPower;
        motor[motorD] = -motorPower;
    }
    else
    {
        motor[motorA] = -motorPower;
        motor[motorD] = motorPower;
    }
    angle = abs(angle);
    resetGyro(S3);
    while (abs(getGyroDegrees(S3)) < angle)
    {}
    motor[motorA] = motor[motorD] = 0;
}

// checks left distance
void checkDistanceLeft(float angle)
{
    resetGyro(S3);
    turnLeft(angle, 0.15);
    while (abs(getGyroDegrees(S3)) < angle)
    {}
    stopD();
}

// checks right distance
void checkDistanceRight(float angle)
{
    resetGyro(S3);
    turnRight(angle, 0.15);
    while (abs(getGyroDegrees(S3)) < angle)
    {}
    stopD();
}

// drives back to initial position
void driveBack(float *distTurn, float *turnAngles, int & pathCount)
{
    rotateRobot(180, 15);
    nMotorEncoder[motorA] = 0;
    resetGyro(S3);
    wait1Msec(50);  // Added delay for EV3 gyro stability

    for(int indexTurn = pathCount ; indexTurn >= 0; indexTurn--)
    {
        float distance = distTurn[indexTurn];


        float angle = turnAngles[indexTurn];

        resetGyro(S3);
        rotateRobot(angle, 25);


        wait1Msec(50);  // Added delay for EV3 gyro stability
        nMotorEncoder[motorA] = 0;

        motor[motorA] = motor[motorD] = 25;
        while(nMotorEncoder[motorA] < distance)
        {
            wait1Msec(10);  // Added delay for EV3 stability
        }
        motor[motorA] = motor[motorD] = 0;
        wait1Msec(50);  // Added delay for EV3 motor stability

    }
}

// set motors to drive forward
void driveforward()
{
	motor[motorA] = motor[motorD] = 50;
}

// reorients towards person
void checkIRDir(float * distTurn, float * turnAngles, int & pathCount)
{
    resetGyro(S3);
    if (getIRBeaconDirection(S2) < 0)
    {
        motor[motorA] = 15;
        motor[motorD] = -15;
        while (getIRBeaconDirection(S2) < 0){}
        turnAngles[pathCount] = -getGyroDegrees(S3);
        distTurn[pathCount] = 0;
        pathCount++;

    }
    else
    {
        motor[motorA] = -15;
        motor[motorD] = 15;
        while (getIRBeaconDirection(S2) > 0){}
        turnAngles[pathCount] = -getGyroDegrees(S3);
        distTurn[pathCount] = 0;
        pathCount++;
    }
    motor[motorA] = motor[motorD] = 0;
}

// collision detection
void colDec(float * distTurn, float * turnAngles, int & pathCount)
{
	displayString(10, "Entered Coldec");
	//float drivedist = 0;

	string coldir = "";


	stopD();

	//drivedist = (abs(nMotorEncoder[motorA]) * (PI*2.75/180));

	distTurn[pathCount] = (abs(nMotorEncoder[motorA]));
	turnAngles[pathCount] = 0;
	pathCount++;

	checkDistanceLeft(90);
	float distLeft = 0;
	distLeft = getUSDistance(S1);
	displayString(4, "%f", getUSDistance(S1));


	checkDistanceRight(180);

	float distRight = 0;
	distRight = getUSDistance(S1);

	checkDistanceLeft(90);


	if (distLeft > distRight)
	{
		coldir = "left";
	}

	else
	{
		coldir = "right";
	}

	do {
		displayString(6, "Entered do while loop ");

		if (coldir == "right")
		{
			checkDistanceRight(90);

			turnAngles[pathCount] = -90;


		}

		else
		{
			checkDistanceLeft(85);

			turnAngles[pathCount] = 85;

		}

		nMotorEncoder[motorA] = 0;
		driveforward();
		while (abs(nMotorEncoder[motorA]) * (PI*2.75/180) < 30)
		{}
		displayString(7, "Exited (PI*2.75/180) < 30 loop ");
		stopD();

		distTurn[pathCount] = (abs(nMotorEncoder[motorA]));
		pathCount++;

		if (coldir == "right")
		{
			checkDistanceLeft(85);

			turnAngles[pathCount] = 85;


		}

		else
		{
			checkDistanceRight(90);

			turnAngles[pathCount] = -90;

		}

		distTurn[pathCount] = 0;

		pathCount++;
	} while (getUSDistance(S1) < 30);

	displayString(8, "Exited the do while loop ");


}


task main()
{
	// configure sensors
	configAllSensors();

	time1[T1] = 0;
	time1[T2] = 0;

	// declare arrays and variables
	int scheduleHours[MAX];
	int scheduleMinutes[MAX];
	int pillCounts[MAX];
	int scheduleSize = 0;
	int scheduleIndex = 0;


	float distTurn[NUMTURN];
	float turnAngles[NUMTURN];

	int pathCount = 0;


	// waits for button press to start
	while (!getButtonPress(buttonAny))
	{}
	while (getButtonPress(buttonAny))
	{}



	//rotation = getGyroDegrees(S3); // when do we want to be initializing these
	//distance = getUSDistance(S1);

	// read pill schedule
	readSchedule(scheduleHours, scheduleMinutes, pillCounts, scheduleSize);

	// repeats pill delivery for 5 minutes
	while(time1[T2] < 5*60000)
	{
		// waits until time to dispense a pill
		while(checkTime(scheduleHours, scheduleMinutes, scheduleIndex, scheduleSize) != 1)
		{}

		// drive up to user
		while (getIRBeaconStrength(S2) < 15 && SensorValue[S4] != 1)
		{
			displayString(5, " %f", getIRBeaconStrength(S2));
		}

		while (getIRBeaconStrength(S2) > 15)
		{
			displayString(5, " %f", getIRBeaconStrength(S2));
			displayString(6, " %f", getIRBeaconStrength(S2));
			checkIRDir(distTurn, turnAngles, pathCount);
			stopD();
			driveforward();

			while (getUSDistance(S1) > 30)
			{}

		//	displayString(9, "Exited getUSDistance(S1) > 30");

			slowD(0.15);
			stopD();

			displayString(11, "%f", getIRBeaconStrength(S2));
			wait1Msec(500);
			displayString(12, "%f", getIRBeaconStrength(S2));


			if (getIRBeaconStrength(S2) > 15)
			{
			colDec(distTurn, turnAngles, pathCount);
			}

			else
			{
				time1[T4] = 0;
				while (time1[T1] < 5000 && SensorValue[S4] != 1){}
			}

			stopD();


		}


		distTurn[pathCount] = nMotorEncoder[motorA];
		turnAngles[pathCount] = 0;

    stopD();

    displayString(13, "User Located");
		// dispense pills
		dispensePills(pillCounts, scheduleIndex);

		// wait for touch sensor
		while(SensorValue(S4)!= 1)
		{}

		// drive back to original spot
		driveBack(distTurn, turnAngles, pathCount);
	}
}
