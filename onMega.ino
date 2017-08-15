/*
This is the code that runs on the Mega.
-----------------------------------------
Created 14/8/2016 by Peter Drew.
*/

//***************Includes*****************
#include "RB_Movement.h"
#include "RB_Motor.h"
#include "EEPROM.h"
#include "NewPing.h" //ultrasonics
#include "Wire.h" //used for compass sensor
#include "Compass.h"

//***************Status LED Colours*******
#define redLed 0
#define blueLed 1 //calibrating
#define greenLed 2 //Primary Program- Main attack/ defence
#define yellowLed 3 //FLASH: position update with ultrasonic
#define aquaLed 4 //Secondary Program
#define purpleLed 5
#define whiteLed 6 //Idle
#define offLed 7

//***************Pinout*******************
//Switches
#define switchRun1Pin 34
#define switchRun2Pin 33
#define buttonCalibratePin 35
#define goalColourPin 36
//Colour Sensors
#define frontColourPin A5
#define leftColourPin A6
#define rightColourPin A7
//IRs
#define leftIRPin A4
#define centreLeftIRPin A2
#define centreRightIRPin A3
#define rightIRPin A1
//Roller
#define rollerDrivePin 10
#define rollerSensePin A0

//***************Class Instances**********
Movement bot(5, 4, 9, 8, 7, 6);
//NewPing frontSonar(23, 22, 300); //(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE)
NewPing rearSonar(25, 24, 300);
NewPing rightSonar(27, 26, 300);
NewPing leftSonar(29, 28, 300);
compass compass1(20, 21, 0x1E);

//***************Field Dimensions*********
const int fieldWidth = 60;
const int fieldLength = 80;
const int goalDepth = 176;

//***************Colour Sensors***********
//Calibration Values
int blackLineThreshold;
int whiteLineThreshold;
int greenColourCalibration;
int blackLineColourCalibration;
int whiteLineColourCalibration;
//Readings
int colourSensorValues[3]; //0 front, 1 left, 2 right

//***************Compass******************
int comVal;
unsigned long compassMillis = 0;

//***************Bot Position*************
int botPosition[2];
//Ultrasonics
const int ultrasonicUpdateCompassTolerance = 2; //the maximium deviation from perpendicular that the robot can be and still update position using the ultrasonics
const int ultrasonicTotalDistanceTolerance = 5;
int ultrasonicValues[4]; //0 front, 1 rear, 2 right, 3 left

//***************IRs**********************
int IRValues[4]; //0 left, 1 centre left, 2 centre right, 2 right

//***************Ball Position************
int ballPosition[2];

//***************Goal Position************
int goalPosition[2];
bool goalVisible = false;

//***************Serial*******************
const int delayBetweenSensorValuesPrints = 1000; //how long to wait between prints of data to serial
unsigned long sensorValuesPrintTime = 0;
bool sensorValuesAutoPrint = false;
String inputStringComputer = "";
bool stringCompleteComputer = false;
char serialComputerdAddTo;

//***************Odroid Serial************
String inputStringOdroid = "";
bool stringCompleteOdroid = false;
char serialOdroidAddTo;

//***************Status LED***************
unsigned long ledFlashTime;
const int ledFlashLength = 400;

//***************Ball Roller**************
int rollerSenseValue;
int rollerBallCapturedThreshold = 205;

//***************Misc*********************
unsigned long currentMillis;
int loopCount = 0;
int minPWM = 0;
bool minPWMFinding = false;

float xdir;
float ydir;

int attackMode = 0;
int oldAttackMode;
int reverseTime;
unsigned long reverseUpdateTime;
int ballIRLostTime;
unsigned long ballIRLostUpdateTime;
int ballRollerLostTime;
unsigned long ballRollerLostUpdateTime;
int startDelayTime;
unsigned long startDelayUpdateTime;
bool kickOff = false;
int goalShootTime;
unsigned long goalShootCheckTime;
bool calibrateButtonPressed = false;
int straightenTime;
unsigned long straightenCheckTime;
int positionTime;
unsigned long positionUpdateTime;

//***************Setup********************
void setup() {
	//set ADC to 16 prescaler to speed up analogRead
	ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2));
	ADCSRA |= bit(ADPS2);
	//Start serial
	Serial.begin(115200);
	Serial.println("Serial begin success"); //initiate and report serial init
	Serial2.begin(115200); //initiate serial for communication with Odroid
	inputStringOdroid.reserve(20);
	//set minPWM
	bot.setMinPWM(0);
	//set pins for status led
	pinMode(30, OUTPUT);
	pinMode(31, OUTPUT);
	pinMode(32, OUTPUT);
	setLed(offLed); //turn off status led
	//set input pins with pullups for switches
	pinMode(switchRun2Pin, INPUT_PULLUP); //calibrate switch
	pinMode(switchRun1Pin, INPUT_PULLUP); //run switch
	pinMode(buttonCalibratePin, INPUT_PULLUP);
	pinMode(goalColourPin, INPUT_PULLUP);
	//set input pins for IR sensors
	pinMode(leftIRPin, INPUT); //left
	pinMode(centreLeftIRPin, INPUT); //centre left
	pinMode(centreRightIRPin, INPUT); //centre right
	pinMode(rightIRPin, INPUT); //right
	//inputs for colour sensors
	pinMode(frontColourPin, INPUT_PULLUP); //front
	pinMode(rightColourPin, INPUT_PULLUP); //right
	pinMode(leftColourPin, INPUT_PULLUP); //left
	//roller
	pinMode(rollerDrivePin, OUTPUT);
	pinMode(rollerSensePin, INPUT);
	//compass setup
	Wire.begin();
	compass1.init();
	EEPROMReadCalibrationValues(); //Read calibratin values from EEPROM
	calculateLineColourThresholds(); //Calculate thresholds for lines using calibration values
	whiteLineThreshold = 405;
	Serial.println("Setup Complete");
}

//***************Loop*********************
void loop() {
	currentMillis = millis();
	readSerialOdroid();
	comVal = compass1.getRelativeBearing();
	updateColourSensors();
	updateIRs();
	updateRollerSensor();
	//updateUltrasonics();
	if (sensorValuesAutoPrint) {
		printSensorValues();
	}
	if (digitalRead(switchRun1Pin) == LOW) {
		kickOff = false;
		setLed(blueLed);
		xdir = cos(currentMillis / 600);
		ydir = sin(currentMillis / 600);
		movedirection(xdir, ydir, 80);
	}
	else if (digitalRead(switchRun2Pin) == LOW) {
		kickOff = true;
		//xdir = cos(currentMillis / 250);
		//ydir = sin(currentMillis / 250);
		setLed(purpleLed);
		movedirection(1, 0, 65);
		delay(500);
		movedirection(0, 1, 65);
		delay(500);
		movedirection(-1, 0, 65);
		delay(500);
		movedirection(0, -1, 65);
		delay(500);
	}
	else {
		if (digitalRead(buttonCalibratePin) == LOW && !calibrateButtonPressed) {
			flashLed(redLed);
			compass1.setZeroBearing();
			calibrateButtonPressed = true;
		}
		else if (digitalRead(buttonCalibratePin) == HIGH) {
			calibrateButtonPressed = false;
		}
		if (minPWMFinding) {
			findMinPWM();
		}
		else {
			minPWM = 0;
		}
		bot.allStop();
		digitalWrite(rollerDrivePin, LOW);
		attackMode = 0;
		setLed(whiteLed);
		doSerialCommands();
	}
}

void rotateStraight() {
	int v1 = sqrt(2 * (-cos(comVal * 1000 / 57296) + 1));
	int v2 = sqrt(2 * (-cos((comVal - 120) * 1000 / 57296) + 1));
	int v3 = sqrt(2 * (-cos((comVal - 240) * 1000 / 57296) + 1));

	bot.manualMotors(v1, v2, v3);
}

void movedirection(float Vx, float Vy, int dirSpeed) {
	float wheelVelocities[3];
	dirSpeed = constrain(abs(dirSpeed), 0, 100);

	wheelVelocities[1] = -(Vx + 1.73205080757*Vy) / 3;
	wheelVelocities[0] = 1.15470053838*Vy + wheelVelocities[1];
	wheelVelocities[2] = -(wheelVelocities[0] + wheelVelocities[1]);

	float highestVelocity = 0;

	for (int i = 0; i < 3; i++) {
		if (wheelVelocities[i] > highestVelocity) {
			highestVelocity = wheelVelocities[i];
		}
	}

	for (int i = 0; i < 3; i++) {
		wheelVelocities[i] = wheelVelocities[i] / highestVelocity;
		wheelVelocities[i] = dirSpeed * wheelVelocities[i];
	}

	bot.manualMotors(wheelVelocities[0], wheelVelocities[1], wheelVelocities[2]);
	//Serial.println("V1" + String(wheelVelocities[0]) + "V2" + String(wheelVelocities[1]) + "V3" + String(wheelVelocities[2]));
	//Serial.println();
	//delay(500);
}

int findMinPWM() {
	minPWM += 5;
	bot.manualMotorsPWM(minPWM, minPWM, minPWM);
	Serial.println(minPWM);
	delay(500);
}

void newMainAttack() {
	switch (attackMode)
	{
	case 0:
		setLed(purpleLed);
		startDelayTime = 0;
		startDelayUpdateTime = currentMillis;
		if (kickOff) {
			attackMode = 1;
		}
		else {
			attackMode = 2;
		}
		break;
	case 1: //kick off
		startDelayTime += currentMillis - startDelayUpdateTime;
		startDelayUpdateTime = currentMillis;
		bot.moveStraight(0, 100);
		if (startDelayTime > 1000) {
			bot.allStop();
			attackMode = 2;
		}
		break;
	case 2:
		setLed(greenLed);
		if (comVal > -25 && comVal < 25) {
			findBallForwards();
		}
		else {
			straightenTime = 0;
			straightenCheckTime = currentMillis;
			attackMode = 4;
		}
		break;
	case 4:
		setLed(yellowLed);
		bot.moveRotate(-comVal / 7);
		straightenTime += currentMillis - straightenCheckTime;
		straightenCheckTime = currentMillis;
		if (straightenTime > 100 && (comVal > -14 && comVal < 14)) {
			bot.allStop();
			attackMode = 2;
		}
		else if (!(comVal > -8 && comVal < 8)) {
			straightenTime = 0;
			straightenCheckTime = currentMillis;
		}
		break;
	case 5:
		setLed(blueLed);
		bot.moveStraight(180, 100);
		positionTime += currentMillis - positionUpdateTime;
		positionUpdateTime = currentMillis;
		if (positionTime > 1000) {
			bot.allStop();
			attackMode = 2;
		}
		break;
	case 6:
		setLed(redLed);
		reverseTime += currentMillis - reverseUpdateTime;
		reverseUpdateTime = currentMillis;
		if (reverseTime > 250) {
			bot.allStop();
			attackMode = 2;
		}
		break;
	default:
		break;
	}
	if (colourSensorValues[0] < whiteLineThreshold) { //check front colour sensor
		oldAttackMode = attackMode;
		bot.moveStraight(180, 100);
		attackMode = 6;
		reverseUpdateTime = currentMillis;
		reverseTime = 0;
	}
	if (colourSensorValues[1] < whiteLineThreshold || colourSensorValues[2] < whiteLineThreshold) { //check rear colour sensors
		oldAttackMode = attackMode;
		bot.moveStraight(0, 100);
		attackMode = 6;
		reverseUpdateTime = currentMillis;
		reverseTime = 0;
	}
}

void mainAttack() {
	switch (attackMode)
	{
	case 0:
		setLed(purpleLed);
		startDelayTime = 0;
		startDelayUpdateTime = currentMillis;
		digitalWrite(rollerDrivePin, HIGH);
		if (kickOff) {
			attackMode = 1;
		}
		else {
			attackMode = 2;
		}
		break;
	case 1: //kick off
		startDelayTime += currentMillis - startDelayUpdateTime;
		startDelayUpdateTime = currentMillis;
		bot.moveStraight(0, 100);
		if (startDelayTime > 1000) {
			bot.allStop();
			attackMode = 3;
		}
		break;
	case 2: //not kicking off
		startDelayTime += currentMillis - startDelayUpdateTime;
		startDelayUpdateTime = currentMillis;
		if (startDelayTime > 1000) {
			attackMode = 3;
		}
		break;
	case 3: //find ball
		setLed(greenLed);
		digitalWrite(rollerDrivePin, HIGH);
		findBallForwards();
		if (ballCaptured()) {
			goalShootCheckTime = currentMillis;
			goalShootTime = 0;
			attackMode = 8;
		}
		break;
	case 4: //aim at goal with compass
		setLed(purpleLed);
		if (comVal < 80 && comVal > -80) {
			bot.allStop();
			attackMode = 5;
		}
		else {
			bot.manualMotors(-10, 10, 40);
		}
	case 5: //find goal
		setLed(redLed);
		aimGoal();
		if (ballCaptured()) {
			ballRollerLostTime = 0;
			ballRollerLostUpdateTime = currentMillis;
		}
		else {
			ballRollerLostTime += currentMillis - ballRollerLostUpdateTime;
			ballRollerLostUpdateTime = currentMillis;
		}
		if (ballRollerLostTime >= 300) {
			attackMode = 3;
		}
		break;
	case 6: //shoot goal
		goalShootTime += currentMillis - goalShootCheckTime;
		goalShootCheckTime = currentMillis;
		if (goalShootTime < 3000) {
			bot.moveStraight(0, 100);
		}
		else if (goalShootTime < 5000) {
			bot.moveStraight(180, 60);
		}
		else {
			bot.allStop();
			attackMode = 3;
		}
		if (colourSensorValues[0] > blackLineThreshold) {
			bot.allStop();
			attackMode = 7;
		}
		break;
	case 7: //hit white line
		setLed(yellowLed);
		bot.moveStraight(180, 35);
		reverseTime += currentMillis - reverseUpdateTime;
		reverseUpdateTime = currentMillis;
		if (reverseTime > 200) {
			bot.allStop();
			attackMode = 3;
		}
		break;
	case 8:
		setLed(blueLed);
		goalShootTime += currentMillis - goalShootCheckTime;
		goalShootCheckTime = currentMillis;
		bot.manualMotors(-45, 45, -comVal * 2);
		if (goalShootTime >= 1500) {
			digitalWrite(rollerDrivePin, LOW);
		}
		else if (goalShootTime >= 3000) {
			attackMode = 3;
		}
		break;
	default:
		break;
	}
	if (colourSensorValues[0] < whiteLineThreshold) {
		oldAttackMode = attackMode;
		attackMode = 7;
		reverseUpdateTime = currentMillis;
		reverseTime = 0;
	}
}

void aimGoal() {
	//rollerMotor.motDriveAuto(100);
	if (goalVisible) {
		if (goalPosition[0] > -200 && goalPosition[0] < 200) {
			if (goalPosition[0] > -75 && goalPosition[0] < 75) {
				goalShootTime = 0;
				goalShootCheckTime = currentMillis;
				attackMode = 6;
			}
			else if (goalPosition[0] < 0) {
				bot.manualMotors(-10, 10, -15);
			}
			else {
				bot.manualMotors(-10, 10, 15);
			}
		}
		else if (goalPosition[0] < 0) {
			bot.manualMotors(-15, 15, -50);
		}
		else {
			bot.manualMotors(-15, 15, 50);
		}
	}
	else {
		bot.manualMotors(-10, 10, 50);
	}
}

void findBallForwards() {
	if (ballVisible()) {
		ballIRLostTime = 0;
		ballIRLostUpdateTime = currentMillis;
		switch (strongestIR())
		{
		case 0:
			bot.manualMotors(-20, 100, 0);
			break;
		case 1:
			bot.moveStraight(0, 100);
			break;
		case 2:
			bot.moveStraight(0, 100);
			break;
		case 3:
			bot.manualMotors(-100, 20, 0);
			break;
		default:
			break;
		}
	}
	else {
		ballIRLostTime += currentMillis - ballIRLostUpdateTime;
		ballIRLostUpdateTime = currentMillis;
		if (ballIRLostTime > 500) {
			positionTime = 0;
			positionUpdateTime = currentMillis;
			attackMode = 5;
		}
		else {
			bot.moveStraight(0, 100);
		}

	}
}

void updateIRs() {
	IRValues[0] = analogRead(leftIRPin);
	IRValues[1] = analogRead(centreLeftIRPin);
	IRValues[2] = analogRead(centreRightIRPin);
	IRValues[3] = analogRead(rightIRPin);
}

bool ballVisible() {
	for (int i = 0; i <= 3; i++) {
		if (IRValues[i] < 900) {
			return true;
		}
	}
	return false;
}

int strongestIR() {
	int lowestIRValue = 1024;
	int lowestIRSensor;
	for (int i = 0; i <= 3; i++) {
		if (IRValues[i] < lowestIRValue) {
			lowestIRSensor = i;
			lowestIRValue = IRValues[i];
		}
	}
	return lowestIRSensor;
}

void updateColourSensors() {
	colourSensorValues[0] = analogRead(frontColourPin);
	colourSensorValues[1] = analogRead(leftColourPin);
	colourSensorValues[2] = analogRead(rightColourPin);
}

void updateRollerSensor() {
	rollerSenseValue = analogRead(rollerSensePin);
}

bool ballCaptured() {
	if (rollerSenseValue > rollerBallCapturedThreshold) {
		return true;
	}
	else {
		return false;
	}
}

void updateUltrasonics() {
	//ultrasonicValues[0] = frontSonar.ping_cm();
	ultrasonicValues[1] = rearSonar.ping_cm();
	ultrasonicValues[2] = rightSonar.ping_cm();
	ultrasonicValues[3] = leftSonar.ping_cm();
}

bool getBotPosition() {
	bot.allStop();
	//make perpendicular
	updateUltrasonics();
	int totalWidth = 0; //by setting to zero forces the later if statement to not be true if real value is never set
	int totalLength = 0;
	int xOrd;
	int yOrd;
	bool xSet = false;
	bool ySet = false;
	if (botIsWidthAligned()) {
		totalWidth = ultrasonicValues[0] + ultrasonicValues[1];
		totalLength = ultrasonicValues[2] + ultrasonicValues[3];
	}
	else if (botIsLengthAligned()) {
		totalLength = ultrasonicValues[0] + ultrasonicValues[1];
		totalWidth = ultrasonicValues[2] + ultrasonicValues[3];
	}
	if (abs(totalWidth - fieldWidth) <= ultrasonicTotalDistanceTolerance) {
		if (comVal > (0 - ultrasonicUpdateCompassTolerance) && comVal < (0 + ultrasonicUpdateCompassTolerance)) {
			xOrd = ultrasonicValues[3]; //0 front, 1 rear, 2 right, 3 left
		}
		else if (comVal > (180 - ultrasonicUpdateCompassTolerance) && comVal < (-180 + ultrasonicUpdateCompassTolerance)) {
			xOrd = ultrasonicValues[2];
		}
		else if (comVal > (90 - ultrasonicUpdateCompassTolerance) && comVal < (90 + ultrasonicUpdateCompassTolerance)) {
			xOrd = ultrasonicValues[1];
		}
		else if (comVal > (-90 - ultrasonicUpdateCompassTolerance) && comVal < (-90 + ultrasonicUpdateCompassTolerance)) {
			xOrd = ultrasonicValues[0];
		}
		xSet = true;
	}
	if (abs(totalLength - fieldLength) <= ultrasonicTotalDistanceTolerance) { //goal width
		if (comVal > (0 - ultrasonicUpdateCompassTolerance) && comVal < (0 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[1]; //0 front, 1 rear, 2 right, 3 left
		}
		else if (comVal > (180 - ultrasonicUpdateCompassTolerance) && comVal < (-180 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[0];
		}
		else if (comVal > (90 - ultrasonicUpdateCompassTolerance) && comVal < (90 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[2];
		}
		else if (comVal > (-90 - ultrasonicUpdateCompassTolerance) && comVal < (-90 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[3];
		}
		ySet = true;
	}
	else if (abs(totalLength - fieldLength - 2 * goalDepth) <= ultrasonicTotalDistanceTolerance) {
		if (comVal > (0 - ultrasonicUpdateCompassTolerance) && comVal < (0 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[1] + goalDepth; //0 front, 1 rear, 2 right, 3 left
		}
		else if (comVal > (180 - ultrasonicUpdateCompassTolerance) && comVal < (-180 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[0] + goalDepth;
		}
		else if (comVal > (90 - ultrasonicUpdateCompassTolerance) && comVal < (90 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[2] + goalDepth;
		}
		else if (comVal > (-90 - ultrasonicUpdateCompassTolerance) && comVal < (-90 + ultrasonicUpdateCompassTolerance)) {
			yOrd = ultrasonicValues[3] + goalDepth;
		}
		ySet = true;
	}
	if (xSet && ySet) {
		botPosition[0] = xOrd;
		botPosition[1] = yOrd;
		flashLed(yellowLed);
	}
}

bool botIsPerpendicular() {
	if (botIsLengthAligned || botIsWidthAligned) {
		return true;
	}
	else {
		return false;
	}
}

bool botIsWidthAligned() {
	if (comVal > (0 - ultrasonicUpdateCompassTolerance) && comVal < (0 + ultrasonicUpdateCompassTolerance)) {
		return true;
	}
	else if (comVal > (180 - ultrasonicUpdateCompassTolerance) && comVal < (-180 + ultrasonicUpdateCompassTolerance)) {
		return true;
	}
	else {
		return false;
	}
}

bool botIsLengthAligned() {
	if (comVal > (90 - ultrasonicUpdateCompassTolerance) && comVal < (90 + ultrasonicUpdateCompassTolerance)) {
		return true;
	}
	else if (comVal > (-90 - ultrasonicUpdateCompassTolerance) && comVal < (-90 + ultrasonicUpdateCompassTolerance)) {
		return true;
	}
	else {
		return false;
	}
}

void calculateLineColourThresholds() {
	blackLineThreshold = (greenColourCalibration + blackLineColourCalibration) / 2;
	whiteLineThreshold = (greenColourCalibration + whiteLineColourCalibration) / 2;
}

void EEPROMReadCalibrationValues() {
	int readBias[3];
	float readScaleFactor[3];
	int eeAddress = 0;
	for (int i = 0; i < 3; i++) {
		EEPROM.get(eeAddress, readBias[i]);
		eeAddress += sizeof(readBias[i]);
	}
	for (int i = 0; i < 3; i++) {
		EEPROM.get(eeAddress, readScaleFactor[i]);
		eeAddress += sizeof(readScaleFactor[i]);
	}
	compass1.importCalibration(readBias, readScaleFactor);
	EEPROM.get(eeAddress, whiteLineColourCalibration);
	eeAddress += sizeof(whiteLineColourCalibration);
	EEPROM.get(eeAddress, greenColourCalibration);
	eeAddress += sizeof(greenColourCalibration);
	EEPROM.get(eeAddress, blackLineColourCalibration);
	eeAddress += sizeof(blackLineColourCalibration);
	EEPROM.get(eeAddress, rollerBallCapturedThreshold);
	eeAddress += sizeof(rollerBallCapturedThreshold);
}

void EEPROMWriteCalibrationValues() {
	int writeBias[3];
	float writeScaleFactor[3];
	int eeAddress = 0;
	compass1.exportCalibration(writeBias, writeScaleFactor);
	for (int i = 0; i < 3; i++) {
		EEPROM.put(eeAddress, writeBias[i]);
		eeAddress += sizeof(writeBias[i]);
	}
	for (int i = 0; i < 3; i++) {
		EEPROM.put(eeAddress, writeScaleFactor[i]);
		eeAddress += sizeof(writeScaleFactor[i]);
	}
	EEPROM.put(eeAddress, whiteLineColourCalibration);
	eeAddress += sizeof(whiteLineColourCalibration);
	EEPROM.put(eeAddress, greenColourCalibration);
	eeAddress += sizeof(greenColourCalibration);
	EEPROM.put(eeAddress, blackLineColourCalibration);
	eeAddress += sizeof(blackLineColourCalibration);
	EEPROM.put(eeAddress, rollerBallCapturedThreshold);
	eeAddress += sizeof(rollerBallCapturedThreshold);
}

void doSerialCommands() {
	while (Serial.available()) {
		char inChar = (char)Serial.read();
		switch (inChar)
		{
		case 'a':
			serialComputerdAddTo = inChar;
			break;
		case 'b':
			serialComputerdAddTo = inChar;
			break;
		case 'c':
			serialComputerdAddTo = inChar;
			break;
		case 'd':
			serialComputerdAddTo = inChar;
			break;
		case 'e':
			serialComputerdAddTo = inChar;
			break;
		case 'f':
			serialComputerdAddTo = inChar;
			break;
		case 'g':
			serialComputerdAddTo = inChar;
			break;
		case 'm':
			serialComputerdAddTo = inChar;
			break;
		case 'r':
			serialComputerdAddTo = inChar;
			break;
		case 's':
			serialComputerdAddTo = inChar;
			break;
		case 'w':
			serialComputerdAddTo = inChar;
			break;
		case 'z':
			serialComputerdAddTo = inChar;
			break;
		case '\n':
			stringCompleteComputer = true;
			break;
		default:
			inputStringComputer += inChar;
			break;
		}
		if (stringCompleteComputer) {
			switch (serialComputerdAddTo)
			{
			case 'c':
				Serial.println("Calibrating Compass");
				Serial.println("This will take approximately 10 seconds.");
				compass1.calibrate();
				delay(500);
				Serial.println("Compass Calibration Done");
				Serial.println("Remember to test and write to EEPROM");
				break;
			case 'e':
				EEPROMWriteCalibrationValues();
				Serial.println("Calibration values written to EEPROM");
				break;
			case 'f':
				EEPROMReadCalibrationValues();
				Serial.println("Calibration values read from EEPROM");
				calculateLineColourThresholds();
				break;
			case 'b':
				blackLineColourCalibration = colourSensorValues[0];
				calculateLineColourThresholds();
				Serial.println("Black Line Value Set");
				Serial.println("Remember to test and write to EEPROM");
				break;
			case 'w':
				whiteLineColourCalibration = colourSensorValues[0];
				calculateLineColourThresholds();
				Serial.println("White Line Value Set");
				Serial.println("Remember to test and write to EEPROM");
				break;
			case 'g':
				greenColourCalibration = colourSensorValues[0];
				calculateLineColourThresholds();
				Serial.println("Green Value Set");
				Serial.println("Remember to test and write to EEPROM");
				break;
			case 'm':
				minPWMFinding = !minPWMFinding;
				break;
			case 'r':
				rollerBallCapturedThreshold = inputStringComputer.toInt();
				Serial.println("Roller Calibration Set");
				Serial.println("Remember to test and write to EEPROM");
				break;
			case 's':
				sensorValuesAutoPrint = !sensorValuesAutoPrint;
				break;
			case 'd':
				sensorValuesAutoPrint = false;
				printCalibrationInformation();
				break;
			case 'z':
				compass1.setZeroBearing();
				Serial.println("Compass Zero Bearing Set");
				break;
			default:
				break;
			}
			stringCompleteComputer = false;
			inputStringComputer = "";
		}
	}
}

void readSerialOdroid() {
	while (Serial2.available()) {
		char inChar = (char)Serial2.read();
		switch (inChar)
		{
		case 'x':
			serialOdroidAddTo = inChar;
			break;
		case 'y':
			serialOdroidAddTo = inChar;
			break;
		case 'r':
			serialOdroidAddTo = inChar;
			break;
		case 'i':
			serialOdroidAddTo = inChar;
			break;
		case 'j':
			serialOdroidAddTo = inChar;
			break;
		case 'k':
			goalVisible = false;
			break;
		case '\n':
			stringCompleteOdroid = true;
			break;
		default:
			inputStringOdroid += inChar;
			break;
		}
		if (stringCompleteOdroid) {
			switch (serialOdroidAddTo)
			{
			case 'x':
				ballPosition[0] = inputStringOdroid.toInt();
				break;
			case 'y':
				ballPosition[1] = inputStringOdroid.toInt();
				break;
			case 'r':
				break;
			case 'i':
				goalPosition[0] = 320 - inputStringOdroid.toInt();
				goalVisible = true;
				break;
			case 'j':
				break;
			default:
				break;
			}
			stringCompleteOdroid = false;
			inputStringOdroid = "";
		}
	}
}

void printCalibrationInformation() {
	Serial.println("");
	Serial.println("");
	Serial.println("**Calibration Data**");
	Serial.println("---------");
	int printBias[3];
	float printScaleFactor[3];
	compass1.exportCalibration(printBias, printScaleFactor);
	Serial.println("Compass Bias");
	for (int i = 0; i < 3; i++) {
		Serial.println(printBias[i]);
	}
	Serial.println("Compass Scale Factor");
	for (int i = 0; i < 3; i++) {
		Serial.println(printScaleFactor[i]);
	}
	Serial.println("---------");
	Serial.print("White Line Calibration Value:");
	Serial.println(whiteLineColourCalibration);
	Serial.print("Green Calibration Value:");
	Serial.println(greenColourCalibration);
	Serial.print("Black Line Calibration Value:");
	Serial.println(blackLineColourCalibration);
	Serial.println("---------");
	Serial.print("Black Line Threshold:");
	Serial.println(blackLineThreshold);
	Serial.print("White Line Threshold:");
	Serial.println(whiteLineThreshold);
	Serial.println("---------");
	Serial.print("Roller Ball Captured Threshold:");
	Serial.println(rollerBallCapturedThreshold);
}

void printSensorValues() {
	loopCount++;
	if (currentMillis - sensorValuesPrintTime >= delayBetweenSensorValuesPrints) {
		Serial.println("");
		Serial.println("");
		Serial.print("Position: ");
		Serial.print(botPosition[0]);
		Serial.print(",");
		Serial.println(botPosition[1]);
		Serial.println("---------");
		Serial.print("Front: ");
		Serial.println(ultrasonicValues[0]);
		Serial.print("Rear: ");
		Serial.println(ultrasonicValues[1]);
		Serial.print("Left: ");
		Serial.println(ultrasonicValues[3]);
		Serial.print("Right: ");
		Serial.println(ultrasonicValues[2]);
		Serial.println("---------");
		Serial.print("Left IR: ");
		Serial.println(IRValues[0]);
		Serial.print("Centre Left IR: ");
		Serial.println(IRValues[1]);
		Serial.print("Centre Right IR: ");
		Serial.println(IRValues[2]);
		Serial.print("Right IR: ");
		Serial.println(IRValues[3]);
		Serial.println("---------");
		Serial.print("Front Colour: ");
		Serial.println(colourSensorValues[0]);
		Serial.print("Left Colour: ");
		Serial.println(colourSensorValues[1]);
		Serial.print("Right Colour: ");
		Serial.println(colourSensorValues[2]);
		Serial.println("---------");
		Serial.print("Relative Bearing: ");
		Serial.println(comVal);
		Serial.println("---------");
		Serial.print("Goal Position: ");
		Serial.println(goalPosition[0]);
		Serial.println("---------");
		Serial.print("Ball Sense: ");
		Serial.println(rollerSenseValue);
		Serial.print("Ball Captured: ");
		Serial.println(ballCaptured());
		Serial.println("---------");
		Serial.print("Loop Count: ");
		Serial.println(loopCount);
		loopCount = 0;
		sensorValuesPrintTime = currentMillis;
	}
}

void flashLed(int ledColour) {
	ledFlashTime = currentMillis;
	switch (ledColour) {
	case 0:
		digitalWrite(30, LOW);
		digitalWrite(31, HIGH);
		digitalWrite(32, HIGH);
		break;
	case 1:
		digitalWrite(30, HIGH);
		digitalWrite(31, LOW);
		digitalWrite(32, HIGH);
		break;
	case 2:
		digitalWrite(30, HIGH);
		digitalWrite(31, HIGH);
		digitalWrite(32, LOW);
		break;
	case 3:
		digitalWrite(30, LOW);
		digitalWrite(31, HIGH);
		digitalWrite(32, LOW);
		break;
	case 4:
		digitalWrite(30, HIGH);
		digitalWrite(31, LOW);
		digitalWrite(32, LOW);
		break;
	case 5:
		digitalWrite(30, LOW);
		digitalWrite(31, LOW);
		digitalWrite(32, HIGH);
		break;
	case 6:
		digitalWrite(30, LOW);
		digitalWrite(31, LOW);
		digitalWrite(32, LOW);
		break;
	case 7:
		digitalWrite(30, HIGH);
		digitalWrite(31, HIGH);
		digitalWrite(32, HIGH);
		break;
	}
}

void setLed(int ledColour) {
	if (currentMillis - ledFlashTime >= ledFlashLength) {
		switch (ledColour) {
		case 0:
			digitalWrite(30, LOW);
			digitalWrite(31, HIGH);
			digitalWrite(32, HIGH);
			break;
		case 1:
			digitalWrite(30, HIGH);
			digitalWrite(31, LOW);
			digitalWrite(32, HIGH);
			break;
		case 2:
			digitalWrite(30, HIGH);
			digitalWrite(31, HIGH);
			digitalWrite(32, LOW);
			break;
		case 3:
			digitalWrite(30, LOW);
			digitalWrite(31, HIGH);
			digitalWrite(32, LOW);
			break;
		case 4:
			digitalWrite(30, HIGH);
			digitalWrite(31, LOW);
			digitalWrite(32, LOW);
			break;
		case 5:
			digitalWrite(30, LOW);
			digitalWrite(31, LOW);
			digitalWrite(32, HIGH);
			break;
		case 6:
			digitalWrite(30, LOW);
			digitalWrite(31, LOW);
			digitalWrite(32, LOW);
			break;
		case 7:
			digitalWrite(30, HIGH);
			digitalWrite(31, HIGH);
			digitalWrite(32, HIGH);
			break;
		}
	}
}
