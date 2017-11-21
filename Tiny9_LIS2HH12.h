#include "Arduino.h"

#ifndef LIS2HH12_h
#define LIS2HH12_h

/*************************** REGISTER MAP ***************************/
#define LIS2HH12_RESERVED0		0x00		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED1		0x01		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED2		0x02		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED3		0x03		// Reserved. Do Not Access. 
#define LIS2HH12_RESERVED4		0x04		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED5		0x05		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED6		0x06		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED7		0x07		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED8		0x08		// Reserved. Do Not Access.
#define LIS2HH12_RESERVED9		0x09		// Reserved. Do Not Access.
#define LIS2HH12_RESERVEDA		0x0A		// Reserved. Do Not Access.
#define LIS2HH12_TEMP_L			0x0B		// Temperature Low Significant Byte (Read)
#define LIS2HH12_TEMP_H			0x0C		// Temperature High Significant Byte (Read)
#define LIS2HH12_RESERVEDE		0x0E		// Reserved. Do Not Access.
#define LIS2HH12_WHO_AM_I		0x0F		// Device ID. (Read)
#define LIS2HH12_ACT_THS		0x1E		// ? (Read/Write)
#define LIS2HH12_ACT_DUR		0x1F		// ? (Read/Write)
#define LIS2HH12_CTRL1			0x20		// Control Register (Read/Write)
#define LIS2HH12_CTRL2			0x21		// Control Register (Read/Write)
#define LIS2HH12_CTRL3			0x22		// Control Register (Read/Write)
#define LIS2HH12_CTRL4			0x23		// Control Register (Read/Write)
#define LIS2HH12_CTRL5			0x24		// Control Register (Read/Write)
#define LIS2HH12_CTRL6			0x25		// Control Register (Read/Write)
#define LIS2HH12_CTRL7			0x26		// Control Register (Read/Write)
#define LIS2HH12_STATUS			0x27		// Status Data Register (Read)
#define LIS2HH12_OUT_X_L		0x28		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_X_H		0x29		// X-Axis_High BYTE (READ)
#define LIS2HH12_OUT_Y_L		0x2A		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_Y_H		0x2B		// X-Axis_High BYTE (READ)
#define LIS2HH12_OUT_Z_L		0x2C		// X-Axis_Low BYTE (READ)
#define LIS2HH12_OUT_Z_H		0x2D		// X-Axis_High BYTE (READ)
#define LIS2HH12_FIFO_CTRL		0x2E		// FIFI Control (Read/Write)
#define LIS2HH12_FIFO_SRC		0x2F		// FIFO ? (Read)
#define LIS2HH12_IG_CFG1		0x30		// Interrupt Generator 1 configuration (Read/Write)
#define LIS2HH12_IG_SRC1		0x31		// Interrupt Generator 1 status Register (Read)
#define LIS2HH12_IG_THS_X1		0x32		// Interrupt generator 1 X Threshold (Read/Write)
#define LIS2HH12_IG_THS_Y1		0x33		// Interrupt Generator 1 Y Threshold (Read/Write)
#define LIS2HH12_IG_THS_Z1		0x34		// Interrupt Generator 1 Z Threshold (Read/Write)
#define LIS2HH12_IG_DUR1		0x35		// Interrupt Generator 1 Duration (Read/Write)
#define LIS2HH12_IG_CFG2		0x36		// Interrupt Generator 2 configuration (Read/Write)
#define LIS2HH12_IG_SRC2		0x37		// Interrupt Generator 2 status Register (Read)
#define LIS2HH12_IG_THS2		0x38		// Interrupt generator 2 Threshold (Read/Write)
#define LIS2HH12_IG_DUR2		0x39		// Interrupt Generator 2 Duration (Read/Write)

#define LIS2HH12_XL_REFERENCE	0x3A		// Reference X Low (Read/Write)
#define LIS2HH12_XH_REFERENCE	0x3B		// Reference X High (Read/Write)
#define LIS2HH12_YL_REFERENCE	0x3C		// Reference Y Low (Read/Write)
#define LIS2HH12_YH_REFERENCE	0x3D		// Reference Y High (Read/Write)
#define LIS2HH12_ZL_REFERENCE	0x3E		// Reference Z Low (Read/Write) 
#define LIS2HH12_ZH_REFERENCE	0x3F		// Reference Z High (Read/Write)


 /************************** INTERRUPT PINS **************************/
#define LIS2HH12_INT1_PIN		0x00		//INT1: 0
#define LIS2HH12_INT2_PIN		0x01		//INT2: 1


 /********************** INTERRUPT BIT POSITION **********************/
#define LIS2HH12_INT_DATA_READY_BIT		0x07
#define LIS2HH12_INT_SINGLE_TAP_BIT		0x06
#define LIS2HH12_INT_DOUBLE_TAP_BIT		0x05
#define LIS2HH12_INT_ACTIVITY_BIT		0x04
#define LIS2HH12_INT_INACTIVITY_BIT		0x03
#define LIS2HH12_INT_FREE_FALL_BIT		0x02
#define LIS2HH12_INT_WATERMARK_BIT		0x01
#define LIS2HH12_INT_OVERRUNY_BIT		0x00

#define LIS2HH12_DATA_READY				0x07
#define LIS2HH12_SINGLE_TAP				0x06
#define LIS2HH12_DOUBLE_TAP				0x05
#define LIS2HH12_ACTIVITY				0x04
#define LIS2HH12_INACTIVITY				0x03
#define LIS2HH12_FREE_FALL				0x02
#define LIS2HH12_WATERMARK				0x01
#define LIS2HH12_OVERRUNY				0x00


 /****************************** ERRORS ******************************/
#define LIS2HH12_OK			1		// No Error
#define LIS2HH12_ERROR		0		// Error Exists

#define LIS2HH12_NO_ERROR	0		// Initial State
#define LIS2HH12_READ_ERROR	1		// Accelerometer Reading Error
#define LIS2HH12_BAD_ARG		2		// Bad Argument


class LIS2HH12
{
public:
	bool status;					// Set When Error Exists 

	byte error_code;				// Initial State
	double gains[3];				// Counts to Gs
	
	LIS2HH12();
	LIS2HH12(int CS);
	void powerOn();
	void readAccel(int* xyx);
	void readAccel(int* x, int* y, int* z);
	void get_Gxyz(double *xyz);
	
	void setTapThreshold(int tapThreshold);
	int getTapThreshold();
	void setAxisGains(double *_gains);
	void getAxisGains(double *_gains);
	void setAxisOffset(int x, int y, int z);
	void getAxisOffset(int* x, int* y, int*z);
	void setTapDuration(int tapDuration);
	int getTapDuration();
	void setDoubleTapLatency(int doubleTapLatency);
	int getDoubleTapLatency();
	void setDoubleTapWindow(int doubleTapWindow);
	int getDoubleTapWindow();
	void setActivityThreshold(int activityThreshold);
	int getActivityThreshold();
	void setInactivityThreshold(int inactivityThreshold);
	int getInactivityThreshold();
	void setTimeInactivity(int timeInactivity);
	int getTimeInactivity();
	void setFreeFallThreshold(int freeFallthreshold);
	int getFreeFallThreshold();
	void setFreeFallDuration(int freeFallDuration);
	int getFreeFallDuration();
	
	bool isActivityXEnabled();
	bool isActivityYEnabled();
	bool isActivityZEnabled();
	bool isInactivityXEnabled();
	bool isInactivityYEnabled();
	bool isInactivityZEnabled();
	bool isActivityAc();
	bool isInactivityAc();
	void setActivityAc(bool state);
	void setInactivityAc(bool state);
	
	bool getSuppressBit();
	void setSuppressBit(bool state);
	bool isTapDetectionOnX();
	void setTapDetectionOnX(bool state);
	bool isTapDetectionOnY();
	void setTapDetectionOnY(bool state);
	bool isTapDetectionOnZ();
	void setTapDetectionOnZ(bool state);
	void setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ);
	
	void setActivityX(bool state);
	void setActivityY(bool state);
	void setActivityZ(bool state);
	void setActivityXYZ(bool stateX, bool stateY, bool stateZ);
	void setInactivityX(bool state);
	void setInactivityY(bool state);
	void setInactivityZ(bool state);
	void setInactivityXYZ(bool stateX, bool stateY, bool stateZ);
	
	bool isActivitySourceOnX();
	bool isActivitySourceOnY();
	bool isActivitySourceOnZ();
	bool isTapSourceOnX();
	bool isTapSourceOnY();
	bool isTapSourceOnZ();
	bool isAsleep();
	
	bool isLowPower();
	void setLowPower(bool state);
	double getRate();
	void setRate(double rate);
	void set_bw(byte bw_code);
	byte get_bw_code();  
	
	bool triggered(byte interrupts, int mask);
	
	byte getInterruptSource();
	bool getInterruptSource(byte interruptBit);
	bool getInterruptMapping(byte interruptBit);
	void setInterruptMapping(byte interruptBit, bool interruptPin);
	bool isInterruptEnabled(byte interruptBit);
	void setInterrupt(byte interruptBit, bool state);
	void setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity);
	void InactivityINT(bool status);
	void ActivityINT(bool status);
	void FreeFallINT(bool status);
	void doubleTapINT(bool status);
	void singleTapINT(bool status);
	
	void getRangeSetting(byte* rangeSetting);
	void setRangeSetting(int val);
	bool getSelfTestBit();
	void setSelfTestBit(bool selfTestBit);
	bool getSpiBit();
	void setSpiBit(bool spiBit);
	bool getInterruptLevelBit();
	void setInterruptLevelBit(bool interruptLevelBit);
	bool getFullResBit();
	void setFullResBit(bool fullResBit);
	bool getJustifyBit();
	void setJustifyBit(bool justifyBit);
	void printAllRegister();
	
private:
	void writeTo(byte address, byte val);
	void writeToI2C(byte address, byte val);
	void writeToSPI(byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	void readFromI2C(byte address, int num, byte buff[]);
	void readFromSPI(byte address, int num, byte buff[]);
	void setRegisterBit(byte regAdress, int bitPos, bool state);
	bool getRegisterBit(byte regAdress, int bitPos);  
	byte _buff[2] ;		//	2 Bytes Buffer
	int _CS = 10;
	bool I2C = true;
	unsigned long SPIfreq = 5000000;
};
void print_byte(byte val);
#endif