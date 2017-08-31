
/*
 *      CAROBOT SwissCHEESE (SC) Library v0.1
 *
 *
 *
 *      created on Aug 2017
 *      by Jacky Lau
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

#include "Arduino.h"

#ifndef CAROBOT_SwissCHEESE_h
#define CAROBOT_SwissCHEESE_h


// Minimum Analog In/Out that each platform have
#define I0 A0
#define I1 A1
#define I2 A2
#define I3 A3
#define I4 A4
#define I5 A5

#define O0 11
#define O1 3
#define O2 6
#define O3 5
#define O4 10
#define O5 9
#define O6 13
#define O7 2

#define M1 9
#define M2 10

#define PWMA 9
#define AIN1 12
#define AIN2 4
#define PWMB 10
#define BIN1 8
#define BIN2 7

// Mega have more I/O
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define I6 A6
#define I7 A7
#define I8 A8
#define I9 A9
#endif

#define SC_MAX 1023
#define SC_X1 0	// identifies the 1x SCGyro model
#define SC_X4 1	// identifies the 4x SCGyro model
#define NORTH 1	// north pole: used in SCHallSensor
#define SOUTH 0	// south pole: used in SCHallSensor

/*
-----------------------------------------------------------------------------
                                Generic Classes
-----------------------------------------------------------------------------
*/

class SCDigital
{
public:
    SCDigital(uint8_t _pin);
    boolean read();

protected:
    uint8_t pin;
};


class SCAnalog
{
public:
    SCAnalog(uint8_t _pin);
    int read();
    boolean increasing();
    boolean decreasing();

protected:
    uint8_t pin;
    int _oldVal;
    boolean _increasing;
    boolean _decreasing;
};

class SCAnalog2
{
public:
    SCAnalog2(uint8_t _pinX, uint8_t _pinY);
    SCAnalog2(uint8_t _pinX, uint8_t _pinY, uint8_t _pinZ);
    int readX();
    int readY();
    int readZ();

protected:
    uint8_t pinX, pinY, pinZ;
};


class SCOutput
{
    public:
    SCOutput (uint8_t _pin);
    void write(int value);
    inline int state() { return _state; }
    void on() {
        write(1023);
        _state = HIGH;
    }
    void off() {
        write(0);
        _state = LOW;
    }
    void blink(int delay);
    void blink(int delay1, int delay2);

protected:
    uint8_t pin;
    int _state;
};

/*
 -----------------------------------------------------------------------------
                                Digital Inputs
 -----------------------------------------------------------------------------
*/


/*      Button      */

class SCButton: public SCDigital
{
	public:
		SCButton(uint8_t _pin);
		boolean readSwitch();
		boolean get(); // added by CAROBOT for ArduBlock
		boolean pressed();
		boolean held();
		boolean released();

	protected:
		boolean _toggleState, _oldState;
		boolean _pressedState, _releasedState;
		boolean _heldState;
        int _heldTime;
        int _millisMark;

		void update();
};


/*      Tilt Sensor     */

class SCTiltSensor: public SCDigital
{
	public:
		SCTiltSensor(uint8_t pin);
};

/*      Touch Sensor        */

class SCTouchSensor : public SCButton
{
	public:
		SCTouchSensor(uint8_t _pin);
};

/*

/*      Ultrasonic Sensor        */

//class SCUltrasonicSensor
//{
//	public:
//		SCUltrasonicSensor(uint8_t _pin);
//};

/*
 -----------------------------------------------------------------------------
                                Analog Inputs
 -----------------------------------------------------------------------------
 */

/*      Potentiometer        */

class SCPotentiometer: public SCAnalog
{
public:
    SCPotentiometer(uint8_t pin);
    int read();
    int readStep(int steps);

protected:
    int _minVal, _maxVal;
    int _mappedVal;
    int _steps;
};


/*      Light Sensor        */

class SCLightSensor : public SCAnalog
{
    public:
        SCLightSensor(uint8_t _pin);
};

/*      Temperature Sensor        */

class SCThermistor : public SCAnalog
{
public:
    SCThermistor(uint8_t _pin);
    float readCelsius();
    float readFahrenheit();

protected:
    const float ADCres = 1023.0;
    const int Beta = 3950;			// Beta parameter
    const float Kelvin = 273.15;	// 0°C = 273.15 K
    const int Rb = 10000;			// 10 kOhm
    const float Ginf = 120.6685;	// Ginf = 1/Rinf
    // Rinf = R0*e^(-Beta/T0) = 4700*e^(-3950/298.15)
};

/*      Hall Sensor        */

class SCHallSensor : public SCAnalog
{
public:
    SCHallSensor(uint8_t _pin);
    boolean polarity();

protected:
    const uint16_t _zeroValue = 512;
};


/*      Joystick        */

class SCJoystick : public SCAnalog2
{
public:
    SCJoystick(uint8_t _pinX, uint8_t _pinY);
    int readX();
    int readY();
protected:
    int _minVal, _maxVal;
    int _mappedVal;
};


/*      Gyro Sensor        */

class SCGyro : public SCAnalog2
{
public:
    SCGyro(uint8_t _pinX, uint8_t _pinY, boolean model);
    void calibrate();
    long readXAxisRate();
    long readYAxisRate();

protected:
    boolean model;

    //const static int _ADCresolution = 4880;	// [mV/count]	multiplierd by 1000 to avoid float numbers
    // minimum sensitivity for the 1x module value (from datasheet is 0.167 mV/deg/s but the TinkerKit module has the outputs amplified 2x)
    //const static int _sensitivity = 334;	// Sensitivity is expressed in mV/degree/seconds, multiplierd by 1000 to avoid float numbers.
    // This value represent the sensitivity of the 1x module. The sensitivity of the 4x module is 4x of this one
    long _sensitivityInCount;	// we obtain the sensitivity expressed in ADC counts
    // [counts/dps]
    int _yZeroVoltage;
    int _xZeroVoltage;
};

/*      Accelerometer        */

class SCAccelerometer : public SCAnalog2
{
public:
    SCAccelerometer(uint8_t _pinX, uint8_t _pinY);
    SCAccelerometer(uint8_t _pinX, uint8_t _pinY, uint8_t _pinZ);
    inline float readXinG() { return (float)(readX() - _zeroOffset)/96; }
    inline float readYinG() { return (float)(readY() - _zeroOffset)/96; }
    inline float readZinG() { return (float)(readZ() - _zeroOffset)/96; }
    int inclination();

protected:
    const float _gain = 1.414;
    const int _zeroOffset = 478;
};

/*
 -----------------------------------------------------------------------------
                                    Outputs
 -----------------------------------------------------------------------------
 */

/*      Buzzer      */

class SCBuzzer : public SCOutput
{
	public:
		SCBuzzer(uint8_t _pin);
};

/*      LED         */

class SCLed : public SCOutput
{
	public:
        SCLed(uint8_t _pin);
        inline void brightness(int value) { write(value); }
};

/*      MosFet      */

class SCMosFet : public SCOutput
{
	public:
		SCMosFet(uint8_t _pin);
};

/*      Relay       */

class SCRelay : public SCOutput
{
	public:
		SCRelay(uint8_t _pin);
};

/*
 -----------------------------------------------------------------------------
                                    Outputs
 -----------------------------------------------------------------------------
 */

 class SCMotor
{
	public:
		SCMotor(uint8_t _pin);
		void speed(int value);
		inline void forward(uint8_t value) { speed(value); }
		inline void backward(uint8_t value) { speed(-value); }
		void stop();

    protected:
        uint8_t pin;
};

#endif

