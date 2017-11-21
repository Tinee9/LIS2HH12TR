/*
Tiny9's LIS2HH12 Library Main Source File
Tiny9_LIS2HH12.cpp

T.J. Schramm @ Tiny9
Created: Nov 14, 2017
Updated: Nov 14, 2017

Modified Bildr LIS2HH12 Source File @ 
to support I2C and SPI Communication (not working yet)

Hardware Resources:
- Arduino Development Board
- Tiny9 Triple Access Accelerometer LIS2HH12

Development Environment Specifics:
Arduino 1.6.8
Tiny9 Triple Axis Accelerometer Breakout - LIS2HH12
Arduino Uno
Arduino Nano
*/

//#include "Arduino.h"
#include "Tiny9_LIS2HH12.h"
#include <Wire.h>
#include <SPI.h>

#define LIS2HH12_DEVICE (30)    // Device Address for LIS2HH12
#define LIS2HH12_TO_READ (1)      // Number of Bytes Read - Two Bytes Per Axis

LIS2HH12::LIS2HH12() {
	status = LIS2HH12_OK;
	error_code = LIS2HH12_NO_ERROR;
	
	gains[0] = 1;//0.00376390;		// Original gain 0.00376390 
	gains[1] = 1;//0.00376009;		// Original gain 0.00376009
	gains[2] = 1;//0.00349265;		// Original gain 0.00349265
	I2C = true;
}
/*int LIS2HH12::scale(int gravity) {
	if(I2C) {
		Wire.begin(LIS2HH12_DEVICE);				// If in I2C Mode Only
		writeTo(LIS2HH12_CTRL4, ((gravity << 4) + 4);
	}
	
	
}*/
/*LIS2HH12::LIS2HH12(int CS) {
	status = LIS2HH12_OK;
	error_code = LIS2HH12_NO_ERROR;
	
	gains[0] = 0.00376390;
	gains[1] = 0.00376009;
	gains[2] = 0.00349265;
	_CS = CS;
	I2C = false;
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	pinMode(_CS, OUTPUT);
	digitalWrite(_CS, HIGH);
}*/

void LIS2HH12::powerOn() {
	if(I2C) {
		Wire.begin(LIS2HH12_DEVICE);				// If in I2C Mode Only
	}
	//LIS2HH12 TURN ON
	//writeTo(LIS2HH12_FIFO_CTRL, 64);
	writeTo(LIS2HH12_CTRL1, 23);
	writeTo(LIS2HH12_CTRL3, 128);
	//writeTo(LIS2HH12_CTRL6, 4);
    //writeTo(LIS2HH12_CTRL4, 4);	
	//writeTo(LIS2HH12_IG_CFG1, 4);	
	//writeTo(LIS2HH12_IG_THS_X1, 10);
	//writeTo(LIS2HH12_IG_DUR1, 10);
	//writeTo(LIS2HH12_IG_CFG2, 16);	
	//writeTo(LIS2HH12_IG_THS2, 1);
	//writeTo(LIS2HH12_IG_DUR2, 1);
	writeTo(LIS2HH12_ACT_THS, 0);	// Activity/Inactivity detection function disabled     
	writeTo(LIS2HH12_ACT_DUR, 0);	// Activity/Inactivity detection function disabled	
	writeTo(LIS2HH12_FIFO_CTRL, 0);
}


/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

void LIS2HH12::readAccel(int *xyz){
	readAccel(xyz, xyz + 1, xyz + 2);
}

void LIS2HH12::readAccel(int *x, int *y, int *z) {
	readFrom(40, 6, _buff);	// Read Accel Data from LIS2HH12
	
	// Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)
	*x = (((int)_buff[1]) << 8) | _buff[0];   
	*y = (((int)_buff[3]) << 8) | _buff[2];
	*z = (((int)_buff[5]) << 8) | _buff[4];
}

void LIS2HH12::get_Gxyz(double *xyz){
	int i;
	int xyz_int[3];
	readAccel(xyz_int);
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * gains[i];
	}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void LIS2HH12::writeTo(byte address, byte val) {
	//if(I2C) {
		writeToI2C(address, val);
	//}
	/*else {
		writeToSPI(address, val);
	}*/
}

/************************ READING NUM BYTES *************************/
/*    Reads Num Bytes. Starts from Address Reg to _buff Array        */
void LIS2HH12::readFrom(byte address, int num, byte _buff[]) {
	//if(I2C) {
		readFromI2C(address, num, _buff);	// If I2C Communication
	//}
	/*else {
		readFromSPI(address, num, _buff);	// If SPI Communication 
	}*/
}

/*************************** WRITE TO I2C ***************************/
/*      Start; Send Register Address; Send Value To Write; End      */
void LIS2HH12::writeToI2C(byte _address, byte _val) {
	Wire.beginTransmission(LIS2HH12_DEVICE); 
	Wire.write(_address);             
	Wire.write(_val);                 
	Wire.endTransmission();         
}

/*************************** READ FROM I2C **************************/
/*                Start; Send Address To Read; End                  */
void LIS2HH12::readFromI2C(byte address, int num, byte _buff[]) {
	Wire.beginTransmission(LIS2HH12_DEVICE);  
	Wire.write(address);             
	Wire.endTransmission();         
	
	Wire.beginTransmission(LIS2HH12_DEVICE); 
	Wire.requestFrom(LIS2HH12_DEVICE, num);  // Request 6 Bytes
	
	int i = 0;
	while(Wire.available())					
	{ 
		_buff[i] = Wire.read();				// Receive Byte
		i++;
	}
	if(i != num){
		status = LIS2HH12_ERROR;
		error_code = LIS2HH12_READ_ERROR;
	}
	Wire.endTransmission();         	
}

/************************** WRITE FROM SPI **************************/
/*         Point to Destination; Write Value; Turn Off              */
/*void LIS2HH12::writeToSPI(byte __reg_address, byte __val) {
  digitalWrite(_CS, LOW);
  SPI.transfer(__reg_address); 
  SPI.transfer(__val); 
  digitalWrite(_CS, HIGH); 
}*/

/*************************** READ FROM SPI **************************/
/*                                                                  */
void LIS2HH12::readFromSPI(byte __reg_address, int num, byte _buff[]) {
  // Read: Most Sig Bit of Reg Address Set
  char _address = 0x80 | __reg_address;
  // If Multi-Byte Read: Bit 6 Set 
  if(num > 1) {
  	_address = _address | 0x40;
  }

  digitalWrite(_CS, LOW);
  SPI.transfer(_address);		// Transfer Starting Reg Address To Be Read  
  for(int i=0; i<num; i++){
    _buff[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS, HIGH);
}

/*************************** RANGE SETTING **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
/*void LIS2HH12::getRangeSetting(byte* rangeSetting) {
	byte _b;
	readFrom(LIS2HH12_DATA_FORMAT, 1, &_b);
	*rangeSetting = _b & B00000011;
}*/

/*void LIS2HH12::setRangeSetting(int val) {
	byte _s;
	byte _b;
	
	switch (val) {
		case 2:  
			_s = B00000000; 
			break;
		case 4:  
			_s = B00000001; 
			break;
		case 8:  
			_s = B00000010; 
			break;
		case 16: 
			_s = B00000011; 
			break;
		default: 
			_s = B00000000;
	}
	readFrom(LIS2HH12_DATA_FORMAT, 1, &_b);
	_s |= (_b & B11101100);
	writeTo(LIS2HH12_DATA_FORMAT, _s);
}*/

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
/*bool LIS2HH12::getSelfTestBit() {
	return getRegisterBit(LIS2HH12_DATA_FORMAT, 7);
}*/

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
/*void LIS2HH12::setSelfTestBit(bool selfTestBit) {
	setRegisterBit(LIS2HH12_DATA_FORMAT, 7, selfTestBit);
}*/

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
/*bool LIS2HH12::getSpiBit() {
	return getRegisterBit(LIS2HH12_DATA_FORMAT, 6);
}*/

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
/*void LIS2HH12::setSpiBit(bool spiBit) {
	setRegisterBit(LIS2HH12_DATA_FORMAT, 6, spiBit);
}*/

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
/*bool LIS2HH12::getInterruptLevelBit() {
	return getRegisterBit(LIS2HH12_DATA_FORMAT, 5);
}*/

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
/*void LIS2HH12::setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(LIS2HH12_DATA_FORMAT, 5, interruptLevelBit);
}*/

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
/*bool LIS2HH12::getFullResBit() {
	return getRegisterBit(LIS2HH12_DATA_FORMAT, 3);
}*/

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
/*void LIS2HH12::setFullResBit(bool fullResBit) {
	setRegisterBit(LIS2HH12_DATA_FORMAT, 3, fullResBit);
}*/

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
/*bool LIS2HH12::getJustifyBit() {
	return getRegisterBit(LIS2HH12_DATA_FORMAT, 2);
}*/

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
/*void LIS2HH12::setJustifyBit(bool justifyBit) {
	setRegisterBit(LIS2HH12_DATA_FORMAT, 2, justifyBit);
}*/

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
/*void LIS2HH12::setTapThreshold(int tapThreshold) {
	tapThreshold = constrain(tapThreshold,0,255);
	byte _b = byte (tapThreshold);
	writeTo(LIS2HH12_THRESH_TAP, _b);  
}*/

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB/* 
// int LIS2HH12::getTapThreshold() {
	// byte _b;
	// readFrom(LIS2HH12_THRESH_TAP, 1, &_b);  
	// return int (_b);
// }

// /****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
// /*                           ~ SET & GET                            */
// /*void LIS2HH12::setAxisGains(double *_gains){
	// int i;
	// for(i = 0; i < 3; i++){
		// gains[i] = _gains[i];
	// }
// }*/
// /*void LIS2HH12::getAxisGains(double *_gains){
	// int i;
	// for(i = 0; i < 3; i++){
		// _gains[i] = gains[i];
	// }
// }*/

// /********************* OFSX, OFSY and OFSZ BYTES ********************/
// /*                           ~ SET & GET                            */
// // OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// // Scale Factor of 15.6mg/LSB
// /*void LIS2HH12::setAxisOffset(int x, int y, int z) {
	// writeTo(LIS2HH12_OFSX, byte (x));  
	// writeTo(LIS2HH12_OFSY, byte (y));  
	// writeTo(LIS2HH12_OFSZ, byte (z));  
// }*/

// /*void LIS2HH12::getAxisOffset(int* x, int* y, int*z) {
	// byte _b;
	// readFrom(LIS2HH12_OFSX, 1, &_b);  
	// *x = int (_b);
	// readFrom(LIS2HH12_OFSY, 1, &_b);  
	// *y = int (_b);
	// readFrom(LIS2HH12_OFSZ, 1, &_b);  
	// *z = int (_b);
// }*/

// /****************************** DUR BYTE ****************************/
// /*                           ~ SET & GET                            */
// // DUR Byte: Contains an Unsigned Time Value Representing the Max Time 
// //  that an Event must be Above the THRESH_TAP Threshold to qualify 
// //  as a Tap Event
// // The scale factor is 625µs/LSB
// // Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
// /*void LIS2HH12::setTapDuration(int tapDuration) {
	// tapDuration = constrain(tapDuration,0,255);
	// byte _b = byte (tapDuration);
	// writeTo(LIS2HH12_DUR, _b);  
// }*/

// /*int LIS2HH12::getTapDuration() {
	// byte _b;
	// readFrom(LIS2HH12_DUR, 1, &_b);  
	// return int (_b);
// }*/

// /************************** LATENT REGISTER *************************/
// /*                           ~ SET & GET                            */
// // Contains Unsigned Time Value Representing the Wait Time from the Detection
// //  of a Tap Event to the Start of the Time Window (defined by the Window 
// //  Register) during which a possible Second Tap Even can be Detected.
// // Scale Factor is 1.25ms/LSB. 
// // A Value of 0 Disables the Double Tap Function.
// // It Accepts a Maximum Value of 255.
// /*void LIS2HH12::setDoubleTapLatency(int doubleTapLatency) {
	// byte _b = byte (doubleTapLatency);
	// writeTo(LIS2HH12_LATENT, _b);  
// }*/

// /*int LIS2HH12::getDoubleTapLatency() {
	// byte _b;
	// readFrom(LIS2HH12_LATENT, 1, &_b);  
	// return int (_b);
// }*/

// /************************** WINDOW REGISTER *************************/
// /*                           ~ SET & GET                            */
// // Contains an Unsigned Time Value Representing the Amount of Time 
// //  After the Expiration of the Latency Time (determined by Latent register)
// //  During which a Second Valid Tape can Begin. 
// // Scale Factor is 1.25ms/LSB. 
// // Value of 0 Disables the Double Tap Function. 
// // It Accepts a Maximum Value of 255.
// /*void LIS2HH12::setDoubleTapWindow(int doubleTapWindow) {
	// doubleTapWindow = constrain(doubleTapWindow,0,255);
	// byte _b = byte (doubleTapWindow);
	// writeTo(LIS2HH12_WINDOW, _b);  
// }*/

// /*int LIS2HH12::getDoubleTapWindow() {
	// byte _b;
	// readFrom(LIS2HH12_WINDOW, 1, &_b);  
	// return int (_b);
// }*/

// /*********************** THRESH_ACT REGISTER ************************/
// /*                          ~ SET & GET                             */
// // Holds the Threshold Value for Detecting Activity.
// // Data Format is Unsigned, so the Magnitude of the Activity Event is Compared 
// //  with the Value is Compared with the Value in the THRESH_ACT Register. 
// // The Scale Factor is 62.5mg/LSB. 
// // Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled. 
// // It Accepts a Maximum Value of 255.
// /*void LIS2HH12::setActivityThreshold(int activityThreshold) {
	// activityThreshold = constrain(activityThreshold,0,255);
	// byte _b = byte (activityThreshold);
	// writeTo(LIS2HH12_THRESH_ACT, _b);  
// }*/

// // Gets the THRESH_ACT byte
// /*int LIS2HH12::getActivityThreshold() {
	// byte _b;
	// readFrom(LIS2HH12_THRESH_ACT, 1, &_b);  
	// return int (_b);
// }*/

// /********************** THRESH_INACT REGISTER ***********************/
// /*                          ~ SET & GET                             */
// // Holds the Threshold Value for Detecting Inactivity.
// // The Data Format is Unsigned, so the Magnitude of the INactivity Event is 
// //  Compared with the value in the THRESH_INACT Register. 
// // Scale Factor is 62.5mg/LSB. 
// // Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled. 
// // It Accepts a Maximum Value of 255.
// /*void LIS2HH12::setInactivityThreshold(int inactivityThreshold) {
	// inactivityThreshold = constrain(inactivityThreshold,0,255);
	// byte _b = byte (inactivityThreshold);
	// writeTo(LIS2HH12_THRESH_INACT, _b);  
// }*/

// /*int LIS2HH12::getInactivityThreshold() {
	// byte _b;
	// readFrom(LIS2HH12_THRESH_INACT, 1, &_b);  
	// return int (_b);
// }*/

// /*********************** TIME_INACT RESIGER *************************/
// /*                          ~ SET & GET                             */
// // Contains an Unsigned Time Value Representing the Amount of Time that
// //  Acceleration must be Less Than the Value in the THRESH_INACT Register
// //  for Inactivity to be Declared. 
// // Uses Filtered Output Data* unlike other Interrupt Functions
// // Scale Factor is 1sec/LSB. 
// // Value Must Be Between 0 and 255. 
// void LIS2HH12::setTimeInactivity(int timeInactivity) {
	// timeInactivity = constrain(timeInactivity,0,255);
	// byte _b = byte (timeInactivity);
	// writeTo(LIS2HH12_TIME_INACT, _b);  
// }

// int LIS2HH12::getTimeInactivity() {
	// byte _b;
	// readFrom(LIS2HH12_TIME_INACT, 1, &_b);  
	// return int (_b);
// }

// ********************** THRESH_FF Register *************************/
                         // ~ SET & GET                            
// // Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// // The Acceleration on all Axes is Compared with the Value in THRES_FF to
// //  Determine if a Free-Fall Event Occurred. 
// // Scale Factor is 62.5mg/LSB. 
// // Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// // Accepts a Maximum Value of 255.
// void LIS2HH12::setFreeFallThreshold(int freeFallThreshold) {
	// freeFallThreshold = constrain(freeFallThreshold,0,255);
	// byte _b = byte (freeFallThreshold);
	// writeTo(LIS2HH12_THRESH_FF, _b);  
// }

// int LIS2HH12::getFreeFallThreshold() {
	// byte _b;
	// readFrom(LIS2HH12_THRESH_FF, 1, &_b);  
	// return int (_b);
// }

//*********************** TIME_FF Register **************************/
//                         ~ SET & GET                            
// Stores an Unsigned Time Value Representing the Minimum Time that the Value 
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
// void LIS2HH12::setFreeFallDuration(int freeFallDuration) {
	// freeFallDuration = constrain(freeFallDuration,0,255);  
	// byte _b = byte (freeFallDuration);
	// writeTo(LIS2HH12_TIME_FF, _b);  
// }

// int LIS2HH12::getFreeFallDuration() {
	// byte _b;
	// readFrom(LIS2HH12_TIME_FF, 1, &_b);  
	// return int (_b);
// }

// ************************* ACTIVITY BITS ***************************/
                                                                
// bool LIS2HH12::isActivityXEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 6); 
// }
// bool LIS2HH12::isActivityYEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 5); 
// }
// bool LIS2HH12::isActivityZEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 4); 
// }
// bool LIS2HH12::isInactivityXEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 2); 
// }
// bool LIS2HH12::isInactivityYEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 1); 
// }
// bool LIS2HH12::isInactivityZEnabled() {  
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 0); 
// }

// void LIS2HH12::setActivityX(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 6, state); 
// }
// void LIS2HH12::setActivityY(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 5, state); 
// }
// void LIS2HH12::setActivityZ(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 4, state); 
// }
// void LIS2HH12::setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
	// setActivityX(stateX);
	// setActivityY(stateY);
	// setActivityZ(stateZ);
// }
// void LIS2HH12::setInactivityX(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 2, state); 
// }
// void LIS2HH12::setInactivityY(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 1, state); 
// }
// void LIS2HH12::setInactivityZ(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 0, state); 
// }
// void LIS2HH12::setInactivityXYZ(bool stateX, bool stateY, bool stateZ) {
	// setInactivityX(stateX);
	// setInactivityY(stateY);
	// setInactivityZ(stateZ);
// }

// bool LIS2HH12::isActivityAc() { 
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 7); 
// }
// bool LIS2HH12::isInactivityAc(){ 
	// return getRegisterBit(LIS2HH12_ACT_INACT_CTL, 3); 
// }

// void LIS2HH12::setActivityAc(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 7, state); 
// }
// void LIS2HH12::setInactivityAc(bool state) {  
	// setRegisterBit(LIS2HH12_ACT_INACT_CTL, 3, state); 
// }

// ************************ SUPPRESS BITS ****************************/
                                                                
// bool LIS2HH12::getSuppressBit(){ 
	// return getRegisterBit(LIS2HH12_TAP_AXES, 3); 
// }
// void LIS2HH12::setSuppressBit(bool state) {  
	// setRegisterBit(LIS2HH12_TAP_AXES, 3, state); 
// }

// *************************** TAP BITS ******************************/
                                                                
// bool LIS2HH12::isTapDetectionOnX(){ 
	// return getRegisterBit(LIS2HH12_TAP_AXES, 2); 
// }
// void LIS2HH12::setTapDetectionOnX(bool state) {  
	// setRegisterBit(LIS2HH12_TAP_AXES, 2, state); 
// }
// bool LIS2HH12::isTapDetectionOnY(){ 
	// return getRegisterBit(LIS2HH12_TAP_AXES, 1); 
// }
// void LIS2HH12::setTapDetectionOnY(bool state) {  
	// setRegisterBit(LIS2HH12_TAP_AXES, 1, state); 
// }
// bool LIS2HH12::isTapDetectionOnZ(){ 
	// return getRegisterBit(LIS2HH12_TAP_AXES, 0); 
// }
// void LIS2HH12::setTapDetectionOnZ(bool state) {  
	// setRegisterBit(LIS2HH12_TAP_AXES, 0, state); 
// }

// void LIS2HH12::setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ) {
	// setTapDetectionOnX(stateX);
	// setTapDetectionOnY(stateY);
	// setTapDetectionOnZ(stateZ);
// }

// bool LIS2HH12::isActivitySourceOnX(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 6); 
// }
// bool LIS2HH12::isActivitySourceOnY(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 5); 
// }
// bool LIS2HH12::isActivitySourceOnZ(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 4); 
// }

// bool LIS2HH12::isTapSourceOnX(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 2); 
// }
// bool LIS2HH12::isTapSourceOnY(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 1); 
// }
// bool LIS2HH12::isTapSourceOnZ(){ 
	// return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 0); 
// }

//************************** ASLEEP BIT *****************************/
                                                                
//bool LIS2HH12::isAsleep(){ 
//	return getRegisterBit(LIS2HH12_ACT_TAP_STATUS, 3); 
//}

//************************* LOW POWER BIT ***************************/
/*                                                                 
bool LIS2HH12::isLowPower(){ 
	return getRegisterBit(LIS2HH12_BW_RATE, 4); 
}
void LIS2HH12::setLowPower(bool state) {  
	setRegisterBit(LIS2HH12_BW_RATE, 4, state); 
}

/*************************** RATE BITS ******************************/
/*                                                                 
double LIS2HH12::getRate(){
	byte _b;
	readFrom(LIS2HH12_BW_RATE, 1, &_b);
	_b &= B00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void LIS2HH12::setRate(double rate){
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) { 
		readFrom(LIS2HH12_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & B11110000);
		writeTo(LIS2HH12_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                            
void LIS2HH12::set_bw(byte bw_code){
	if((bw_code < LIS2HH12_BW_0_05) || (bw_code > LIS2HH12_BW_1600)){
		status = false;
		error_code = LIS2HH12_BAD_ARG;
	}
	else{
		writeTo(LIS2HH12_BW_RATE, bw_code);
	}
}

byte LIS2HH12::get_bw_code(){
	byte bw_code;
	readFrom(LIS2HH12_BW_RATE, 1, &bw_code);
	return bw_code;
}




/************************* TRIGGER CHECK  ***************************/
/*                                                                 
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, LIS2HH12_SINGLE_TAP);
bool LIS2HH12::triggered(byte interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/*
 LIS2HH12_DATA_READY
 LIS2HH12_SINGLE_TAP
 LIS2HH12_DOUBLE_TAP
 LIS2HH12_ACTIVITY
 LIS2HH12_INACTIVITY
 LIS2HH12_FREE_FALL
 LIS2HH12_WATERMARK
 LIS2HH12_OVERRUNY



byte LIS2HH12::getInterruptSource() {
	byte _b;
	readFrom(LIS2HH12_INT_SOURCE, 1, &_b);
	return _b;
}

bool LIS2HH12::getInterruptSource(byte interruptBit) {
	return getRegisterBit(LIS2HH12_INT_SOURCE,interruptBit);
}

bool LIS2HH12::getInterruptMapping(byte interruptBit) {
	return getRegisterBit(LIS2HH12_INT_MAP,interruptBit);
}

/*********************** INTERRUPT MAPPING **************************/
 //        Set the Mapping of an Interrupt to pin1 or pin2         
// eg: setInterruptMapping(LIS2HH12_INT_DOUBLE_TAP_BIT,LIS2HH12_INT2_PIN);
/*void LIS2HH12::setInterruptMapping(byte interruptBit, bool interruptPin) {
	setRegisterBit(LIS2HH12_INT_MAP, interruptBit, interruptPin);
}

void LIS2HH12::setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity) {
	if(single_tap == 1) {
		setInterruptMapping( LIS2HH12_INT_SINGLE_TAP_BIT,   LIS2HH12_INT1_PIN );}
	else if(single_tap == 2) {
		setInterruptMapping( LIS2HH12_INT_SINGLE_TAP_BIT,   LIS2HH12_INT2_PIN );}

	if(double_tap == 1) {
		setInterruptMapping( LIS2HH12_INT_DOUBLE_TAP_BIT,   LIS2HH12_INT1_PIN );}
	else if(double_tap == 2) {
		setInterruptMapping( LIS2HH12_INT_DOUBLE_TAP_BIT,   LIS2HH12_INT2_PIN );}

	if(free_fall == 1) {
		setInterruptMapping( LIS2HH12_INT_FREE_FALL_BIT,   LIS2HH12_INT1_PIN );}
	else if(free_fall == 2) {
		setInterruptMapping( LIS2HH12_INT_FREE_FALL_BIT,   LIS2HH12_INT2_PIN );}

	if(activity == 1) {
		setInterruptMapping( LIS2HH12_INT_ACTIVITY_BIT,   LIS2HH12_INT1_PIN );}
	else if(activity == 2) {
		setInterruptMapping( LIS2HH12_INT_ACTIVITY_BIT,   LIS2HH12_INT2_PIN );}

	if(inactivity == 1) {
		setInterruptMapping( LIS2HH12_INT_INACTIVITY_BIT,   LIS2HH12_INT1_PIN );}
	else if(inactivity == 2) {
		setInterruptMapping( LIS2HH12_INT_INACTIVITY_BIT,   LIS2HH12_INT2_PIN );}
}

bool LIS2HH12::isInterruptEnabled(byte interruptBit) {
	return getRegisterBit(LIS2HH12_INT_ENABLE,interruptBit);
}

void LIS2HH12::setInterrupt(byte interruptBit, bool state) {
	setRegisterBit(LIS2HH12_INT_ENABLE, interruptBit, state);
}

void LIS2HH12::singleTapINT(bool status) {
	if(status) {
		setInterrupt( LIS2HH12_INT_SINGLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( LIS2HH12_INT_SINGLE_TAP_BIT, 0);
	}
}
void LIS2HH12::doubleTapINT(bool status) {
	if(status) {
		setInterrupt( LIS2HH12_INT_DOUBLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( LIS2HH12_INT_DOUBLE_TAP_BIT, 0);		
	}	
}
void LIS2HH12::FreeFallINT(bool status) {
	if(status) {
		setInterrupt( LIS2HH12_INT_FREE_FALL_BIT,  1);
	}
	else {
		setInterrupt( LIS2HH12_INT_FREE_FALL_BIT,  0);
	}	
}
void LIS2HH12::ActivityINT(bool status) {
	if(status) {
		setInterrupt( LIS2HH12_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( LIS2HH12_INT_ACTIVITY_BIT,   0);
	}
}
void LIS2HH12::InactivityINT(bool status) {
	if(status) {
		setInterrupt( LIS2HH12_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( LIS2HH12_INT_INACTIVITY_BIT, 0);
	}
}

void LIS2HH12::setRegisterBit(byte regAdress, int bitPos, bool state) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.  
	} 
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);  
}

bool LIS2HH12::getRegisterBit(byte regAdress, int bitPos) {
	byte _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}*/

/********************************************************************/
                                                                 
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
/*void LIS2HH12::printAllRegister() {
	byte _b;
	Serial.print("0x00: ");
	readFrom(0x00, 1, &_b);
	print_byte(_b);
	Serial.println("");
	int i;
	for (i=29;i<=57;i++){
		Serial.print("0x");
		Serial.print(i, HEX);
		Serial.print(": ");
		readFrom(i, 1, &_b);
		print_byte(_b);
		Serial.println("");    
	}
}

void print_byte(byte val){
	int i;
	Serial.print("B");
	for(i=7; i>=0; i--){
		Serial.print(val >> i & 1, BIN);
	}
} */