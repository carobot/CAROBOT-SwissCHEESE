#include "CAROBOT_SwissCHEESE.h"

#if USB_VID == 0x2341 && USB_PID == 0x803C
#include <Esplora.h>
#endif

/*
 -----------------------------------------------------------------------------
                                    Generals
 -----------------------------------------------------------------------------
*/

/*      Digital Input       */

SCDigital::SCDigital(uint8_t _pin)
{
    pin = _pin;
    pinMode(pin, INPUT);
}

boolean SCDigital::read() {

    boolean val;

    #if USB_VID == 0x2341 && USB_PID == 0x803C
        int value = Esplora.readTK(pin);
        if (value < 128)
            val = 0;
        else
            val = 1;
    #else
        val = digitalRead(pin);
    #endif

    return val;
}


/*      Analog Input        */

SCAnalog::SCAnalog(uint8_t _pin)
{
    pin = _pin;
}

int SCAnalog::read() {

    int val;

    #if USB_VID == 0x2341 && USB_PID == 0x803C
        val = Esplora.readTK(pin);
    #else
        val = analogRead(pin);
    #endif

    if (val > _oldVal)
    {
      _increasing = true;
      _decreasing = false;
    }

    if (val < _oldVal)
    {
      _increasing = false;
      _decreasing = true;
    }

    _oldVal = val;

    return val;
}

boolean SCAnalog::increasing() {
    SCAnalog::read();
    return _increasing;
}

boolean SCAnalog::decreasing() {
    SCAnalog::read();
    return _decreasing;
}

/*      Analog Input with two connectors       */

SCAnalog2::SCAnalog2(uint8_t _pinX, uint8_t _pinY)
{
    pinX = _pinX;
    pinY = _pinY;
}

SCAnalog2::SCAnalog2(uint8_t _pinX, uint8_t _pinY, uint8_t _pinZ)
{
    pinX = _pinX;
    pinY = _pinY;
    pinZ = _pinZ;
}

int SCAnalog2::readX() {

    int val;

    #if USB_VID == 0x2341 && USB_PID == 0x803C
        val = Esplora.readTK(pinX);
    #else
        val = analogRead(pinX);
    #endif

    return val;
}

int SCAnalog2::readY() {

    int val;

    #if USB_VID == 0x2341 && USB_PID == 0x803C
        val = Esplora.readTK(pinY);
    #else
        val = analogRead(pinY);
    #endif

    return val;
}

int SCAnalog2::readZ() {

    int val;

    #if USB_VID == 0x2341 && USB_PID == 0x803C
        val = Esplora.readTK(pinZ);
    #else
        val = analogRead(pinZ);
    #endif

    return val;
}

/*      Output       */

SCOutput::SCOutput(uint8_t _pin)
{
    pin = _pin;
	_state = LOW;
	pinMode(pin, OUTPUT);
}

void SCOutput::write(int value)
{
    if( value <= SC_MAX && value >= 0 )
        analogWrite(pin, value * 0.25);
    else
        return;
}

void SCOutput::blink(int del)
{
    on();
    delay(del);
    off();
    delay(del);
}

void SCOutput::blink(int del1, int del2)
{
    on();
    delay(del1);
    off();
    delay(del2);
}

/*
 -----------------------------------------------------------------------------
                                Digital Inputs
 -----------------------------------------------------------------------------
 */

/*      Button      */

SCButton::SCButton(uint8_t _pin) : SCDigital(_pin)
{
	_toggleState = LOW;
	_oldState = LOW;
	_pressedState = LOW;
	_releasedState = LOW;
	_heldState = LOW;
    _heldTime = 500;
}

void SCButton::update() {
  boolean newState = SCButton::read();
  if (newState != _oldState) {
    // pressed?
    if (newState == HIGH) {
      _pressedState = true;
    }
    else {
      _releasedState = true;
     _toggleState = !_toggleState;
    }

    _oldState = newState;
    delay(50); // debouncing
  }

  else {

      int timeDiff = millis() - _millisMark;

      if(newState == HIGH && _oldState == HIGH && timeDiff > _heldTime) {
  		_heldState = true;
  	} else {
  		_heldState = false;
  	}


  }
}

boolean SCButton::get()
{
	return SCButton::pressed();
}

boolean SCButton::readSwitch()
{
	SCButton::update();
	return _toggleState;
}

boolean SCButton::pressed()
{
	SCButton::update();

	if(_pressedState == true)
	{
        _millisMark = millis();
		_pressedState = false;
		return true;
	}
	else
		return false;
}

boolean SCButton::released()
{
	SCButton::update();

	if(_releasedState == true)
	{
		_releasedState = false;
		return true;
	}
	else
		return false;
}

boolean SCButton::held()
{
	SCButton::update();
	return _heldState;
}


/*      Tilt Sensor         */

SCTiltSensor::SCTiltSensor(uint8_t _pin) : SCDigital (_pin){}

/*      Touch Sensor        */

 SCTouchSensor::SCTouchSensor(uint8_t _pin) : SCButton(_pin) {}

/*
 -----------------------------------------------------------------------------
                                Analog Inputs
 -----------------------------------------------------------------------------
 */

/*      Potentiometer       */

SCPotentiometer::SCPotentiometer(uint8_t _pin) : SCAnalog(_pin)

{
	pin = _pin;
	_minVal = 1023;
	_maxVal = 0;
}

int SCPotentiometer::read()
{

    int val = SCAnalog::read();

	if (val < _minVal) {_minVal = val;}
	if (val > _maxVal) {_maxVal = val;}

	_mappedVal = map(val, _minVal, _maxVal, 0, 1023);
	_mappedVal = constrain(_mappedVal, 0, 1023);

	return _mappedVal;
}

int SCPotentiometer::readStep(int steps) {

	_steps = steps;

	int step  = floor(map(read(), 0, 1023, 0, _steps));

	return step;
}


/*      Light Sensor        */

SCLightSensor::SCLightSensor(uint8_t _pin) : SCAnalog(_pin){}

/*      Temperature Sensor       */

SCThermistor::SCThermistor(uint8_t _pin) : SCAnalog(_pin) {}

float SCThermistor::readCelsius()
{
	float Rthermistor = Rb * (ADCres / SCThermistor::read() - 1);
	float _temperatureC = Beta / (log( Rthermistor * Ginf )) ;

	return _temperatureC - Kelvin;
}

float SCThermistor::readFahrenheit()
{
	float _temperatureF = (SCThermistor::readCelsius() * 9.0)/ 5.0 + 32.0; ;

	return _temperatureF;
}


/*      Hall Sensor     */

SCHallSensor::SCHallSensor(uint8_t _pin) : SCAnalog(_pin) {}

boolean SCHallSensor::polarity()
{
	int value = read();
	if( value >= _zeroValue  )
		return NORTH;
	else
		return SOUTH;
}

/*      Joystick        */

SCJoystick::SCJoystick(uint8_t _pinX, uint8_t _pinY) : SCAnalog2 (_pinX, _pinY){}

int SCJoystick::readX()
{

    int val = SCAnalog2::readX();

	if (val < _minVal) {_minVal = val;}
	if (val > _maxVal) {_maxVal = val;}

	_mappedVal = map(val, _minVal, _maxVal, 0, 1023);
	_mappedVal = constrain(_mappedVal, 0, 1023);

	return _mappedVal;
}

int SCJoystick::readY()
{

    int val = SCAnalog2::readY();

	if (val < _minVal) {_minVal = val;}
	if (val > _maxVal) {_maxVal = val;}

	_mappedVal = map(val, _minVal, _maxVal, 0, 1023);
	_mappedVal = constrain(_mappedVal, 0, 1023);

	return _mappedVal;
}


/*      Gyro        */

SCGyro::SCGyro(uint8_t _pinX, uint8_t _pinY, boolean model) : SCAnalog2 (_pinX, _pinY)
{

 	_sensitivityInCount = 14633;  // 4.88mV / (0.167mV/dps * 2)

 	if(model == SC_X4)
 		_sensitivityInCount /= 4;

    // default values
    _xZeroVoltage = 503;	// 2.46V expressed in ADC counts
    _yZeroVoltage = 503;
}

void SCGyro::calibrate()
{
	_xZeroVoltage = 0;
	_yZeroVoltage = 0;

	for (uint8_t i=0; i<50; i++)
   {
   	_yZeroVoltage += readY();
   	_xZeroVoltage += readX();
   	delay(20);
   }
   _yZeroVoltage /= 50;
   _xZeroVoltage /= 50;
}

long SCGyro::readXAxisRate()
{
 	return ((long)(readX() - _xZeroVoltage) * _sensitivityInCount) / 1000;
}

long SCGyro::readYAxisRate()
{
	return ((long)(readY() - _yZeroVoltage) * _sensitivityInCount) / 1000;
}


 /*
  * Accelerometer Class and Methods
  */

SCAccelerometer::SCAccelerometer(uint8_t _pinX, uint8_t _pinY) : SCAnalog2(_pinX,_pinY){}
SCAccelerometer::SCAccelerometer(uint8_t _pinX, uint8_t _pinY, uint8_t _pinZ) : SCAnalog2(_pinX,_pinY,_pinZ){}

int SCAccelerometer::inclination()
{
	int xVal = readX() - _zeroOffset;
	int yVal = readY() - _zeroOffset;

	if(xVal <= 96 && yVal <= 96)
	{
		int inclination = atan2(xVal, yVal)*180/M_PI;
		return (int) inclination;
	} else {
		return 0;
	}
}

/*
-----------------------------------------------------------------------------
                                        Outputs
-----------------------------------------------------------------------------
*/

/* Buzzer */

SCBuzzer::SCBuzzer(uint8_t _pin) : SCOutput(_pin) {}

/* LED */

SCLed::SCLed(uint8_t _pin) : SCOutput(_pin) {}

/* MosFet */

SCMosFet::SCMosFet(uint8_t _pin) : SCOutput(_pin) {}

/* Relay */

SCRelay::SCRelay(uint8_t _pin) : SCOutput(_pin) {}

/*
-----------------------------------------------------------------------------
                                        Motor
-----------------------------------------------------------------------------
*/

SCMotor::SCMotor(uint8_t _pin)
{
    pin = _pin;
    if (pin == M1) {
        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        analogWrite(M1, 0);
    } else if (pin == M2) {
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        analogWrite(M2, 0);
    }
}

void SCMotor::speed(int speed)
{
    speed = constrain(speed, -255, 255);
    if (pin == M1) {
        if (speed > 0) {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            analogWrite(M1, abs(speed));
        } else {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            analogWrite(M1, abs(speed));
        }
    } else if (pin == M2) {
        if (speed > 0) {
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            analogWrite(M2, abs(speed));
        } else {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
            analogWrite(M2, abs(speed));
        }
    }
}

void SCMotor::stop()
{
    if (pin == M1) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        analogWrite(M1, 0);
    } else if (pin == M2) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, HIGH);
        analogWrite(M2, 0);
    }
}

