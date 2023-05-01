const tSensors TOUCH_SENSOR = S1;
const tSensors ULTRASONIC_SENSOR = S2;
const tSensors COLOUR_SENSOR = S3;
const tSensors GYRO_SENSOR = S4;

const float GYRO_TOLERANCE = 5;
//used to calibrate gyro: if the gyro exibits a change of magnitude of more than 5 degrees within 1 second while stationary, the gyro is recalibrated

const float RADIUS = 2.75;
const int MOTORPOWER = 40;
const int ROTATEANGLE = 90;
const int DISTANCE = 10;
const int ENC_LIMIT = DISTANCE*180/(PI*RADIUS);
//distance here depends on how far we want to place the object away from the robot
//this enconder limit is for robot pickup/dropoff

const int SAFE_ULTRASONIC_DISTANCE = 40;
//for obstacle detection

const int EXITDISTANCE = 60;
//for parking

const int TOTAL_DELIVERIES = 1;
//changeable value for demonstrations. After all deliveries are made, robot exits course and parks itself.

void preparationSequence();
bool colourDecisions(bool & objectInGripper);
void pathCorrection();
void smartRotateAngle(int angle, int motorPower);
void obstacleCheck(int maxSafeDistance);
void gripperPickUp (int pickupOrDropoff);
void parkRobot();


//main assumption: the robot starts on a white line in the course facing the intended direction(travelling clockwise in a loop)
task main()
{
	preparationSequence();
	//initiate all sensors
	time1[T1]=0;

	int deliveriesMade = 0;

	while(deliveriesMade<TOTAL_DELIVERIES){
		bool deliveredStatus = false;
		bool objectInGripper = false;
		while(!deliveredStatus){
			deliveredStatus = colourDecisions(objectInGripper);
			obstacleCheck(SAFE_ULTRASONIC_DISTANCE);
		}

		deliveriesMade++;
	}

	parkRobot();
	//the robot exits the course after making its final delivery

}

//calibrates sensors and samples gyro before and after 1 second delay to ensure gyro is properly calibrated
void preparationSequence()
{

	SensorType[TOUCH_SENSOR]=sensorEV3_Touch;
	SensorType[ULTRASONIC_SENSOR]=sensorEV3_Ultrasonic;

	SensorType[GYRO_SENSOR]=sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[GYRO_SENSOR]=modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[GYRO_SENSOR]=modeEV3Gyro_RateAndAngle;
	wait1Msec(50);

	SensorType[COLOUR_SENSOR]=sensorEV3_Color;
	wait1Msec(50);
	SensorMode[COLOUR_SENSOR]=modeEV3Color_Color;
	wait1Msec(50);

	bool sensorCalibration = false;

	while (sensorCalibration == false){
		motor[motorA]=motor[motorD]=0;
		resetGyro(S4);

		int initialGyro = getGyroDegrees(S4);
		wait1Msec(1000);
		int finalGyro = getGyroDegrees(GYRO_SENSOR);

		float gyroDiff = fabs(initialGyro-finalGyro);
		if (gyroDiff < GYRO_TOLERANCE)
			sensorCalibration = true;
	}

}

//decides robot action based on color detected by the color sensor, decision is redecided each time the function is run
bool colourDecisions(bool & objectInGripper){
	//assumes color sensors are already calibrated

	//drive the robot forwards until a color other than black is detected
	motor[motorA]=motor[motorD]=MOTORPOWER;
	while (SensorValue(COLOUR_SENSOR)==(int)colorBlack){}

	//If a color code representing pickup or dropoff is not detected, correct the course of the robot
	if(SensorValue(COLOUR_SENSOR)!=(int)colorBlue && SensorValue(COLOUR_SENSOR)!=(int)colorRed)
	{
		pathCorrection();
	}

	else if (SensorValue(COLOUR_SENSOR)==(int)colorBlue && objectInGripper==false)
	{
		//initiate pickup
		gripperPickUp(1);
		objectInGripper = true;
	}
	else if (SensorValue(COLOUR_SENSOR)==(int)colorRed && objectInGripper==true)
	{
		//initiate dropoff
		gripperPickUp(0);
		objectInGripper = false;
		//a dropoff has been completed, meaning a full delivery has been made
		return true;
	}

	//a full delivery has not been made yet
	return false;
}

//a decision making path correction function. If green is detected, rotate inwards until white is detected.
//Otherwise, if yellow is detected
void pathCorrection(){

		while (SensorValue(COLOUR_SENSOR)==(int)colorGreen){
			//rotate in small increments to the right
			smartRotateAngle(3, MOTORPOWER);
		}
		while (SensorValue(COLOUR_SENSOR)==(int)colorYellow){
		//rotate left until white
			smartRotateAngle(-3, MOTORPOWER);
		}

}

//slows down angle rotation after getting close to angle
void smartRotateAngle(int angle, int motorPower)
{

	resetGyro(GYRO_SENSOR);
	if(angle > 0){
		motor[motorA]=-motorPower;
		motor[motorD]=motorPower;
	}else{
		motor[motorA]=motorPower;
		motor[motorD]=-motorPower;
	}
	float halfAngle = abs(angle) * 0.3;
	//creates variable for first section of turn
	while(abs(getGyroDegrees(GYRO_SENSOR)) < halfAngle) {}
	//first section of turn angle turns at motorPower

	int slowMotorPower = motorPower * 0.4;
	//creates variable for slower motorPower

	if(angle > 0){
		motor[motorA]=-slowMotorPower;
		motor[motorD]=slowMotorPower;
	}else{
		motor[motorA]=slowMotorPower;
		motor[motorD]=-slowMotorPower;
	}
	//second half of turn is slower so more accurate

	float turnAngle = abs(angle)-3;
	//remove 3 degrees to account for Gyro inaccuracy
	while(abs(getGyroDegrees(GYRO_SENSOR)) < turnAngle) {}
	motor[motorA]=motor[motorD] = 0;
}

//moves robot back a bit and stops robot until obstacles are not detected
void obstacleCheck(int maxSafeDistance){
	if (SensorValue(ULTRASONIC_SENSOR)< maxSafeDistance || SensorValue(TOUCH_SENSOR)==1){
		motor[motorA]=motor[motorD]=-15;
		wait1Msec(1000);
		motor[motorA]=motor[motorD]=0;
	}
	if (SensorValue(ULTRASONIC_SENSOR)< maxSafeDistance)
		obstacleCheck(SAFE_ULTRASONIC_DISTANCE);

}

//1 means pickup, 0 means dropoff
void gripperPickUp (int pickupOrDropoff)
{
	resetGyro(GYRO_SENSOR);
	wait1Msec(50);

	wait1Msec(50);
	nMotorEncoder[motorA] = 0;

	motor[motorA] = motor[motorD] = 0;

	if(pickupOrDropoff==1){
		//opens up gripper (automatically drops off object if it is in the claw)
		motor[motorB]= 5;
		wait1Msec(5000);
		motor[motorB] = 0;
	}

	smartRotateAngle(ROTATEANGLE, MOTORPOWER); //rotate 90 degrees to the right at predesignated motorpower

	motor[motorA] = motor[motorD] = MOTORPOWER;
	while (nMotorEncoder[motorA] < ENC_LIMIT){}  //ENC_LIMIT will need to be determined before this code can work

	motor[motorA] = motor[motorD] = 0;
	if(pickupOrDropoff==0){
		//opens up gripper (automatically drops off object if it is in the claw)
		motor[motorB]= 5;
		wait1Msec(5000);
		motor[motorB] = 0;
	}

	//intention to pickup(blue)
	if(pickupOrDropoff == 1){
		motor[motorB] = -5;  //assumed that the gripper is activated by motorB, and turns on for 2 seconds before it stops
		wait1Msec(5000);
		motor[motorB] = 0;  //may need to be changed if there is a better code for gripper, since the object might fall when the gripper motor stops
	}

	nMotorEncoder [motorA] = 0;

	motor[motorA] = motor[motorD] = -MOTORPOWER; //robot reverses back on the same path, this time the ENC_LIMIT will be negative since robot is going backwards
	while (nMotorEncoder[motorA] > -ENC_LIMIT){}

	motor[motorA] = motor[motorD] = 0;

	if(pickupOrDropoff == 0){
		motor[motorB]= -5;
		wait1Msec(5000);
		motor[motorB] = 0;
	}
	//close gripper
	smartRotateAngle(-ROTATEANGLE, MOTORPOWER);
}

//leave course, ending program
void parkRobot()
{
	nMotorEncoder[motorA] = 0;
	motor[motorA]=motor[motorD] = MOTORPOWER;
	while (nMotorEncoder[motorA] < EXITDISTANCE*180.0/(PI*RADIUS)){}
	motor[motorA]=motor[motorD]=0;

	float duration = time1[T1]/1000.0;
	displayString(5, "Deliveries done, robot parked.");
	displayString(6, "%f.2",duration);

}
