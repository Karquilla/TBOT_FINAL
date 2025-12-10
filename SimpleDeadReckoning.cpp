/**********************************************************************;
* Project           : RES_Class, Localization with Dead Reckoning 
*
* Program name      : SimpleDeadReckoning.cpp
*
* Author            : Jong-Hoon Kim
*
* Date created      : 11/03/2025
*
* Purpose           : Localization of a mini TurtleBot
*
* Revision History  :
*
* Date        Author      Ref    Revision (Date in MMDDYYYY format) 
* MMDDYYYY    name      1  v-xx   revision note. 
*
*********************************************************************/

#include "SimpleDeadReckoning.h"
#include <Arduino.h>
#include <math.h>

SimpleDeadReckoning::SimpleDeadReckoning(float argEncoderValuePer360Rotation, float argWheelRadius, float argDistanceBetweenTwoWheels){
  _eValue = (360.0/argEncoderValuePer360Rotation) * (3.141592 / 180.0);
  _wRadius = argWheelRadius;
  _wDistance = argDistanceBetweenTwoWheels;
  _unit = 0;
}

SimpleDeadReckoning::SimpleDeadReckoning(float argEncoderValuePer360Rotation, float argWheelRadius, float argDistanceBetweenTwoWheels, int argDistanceUnit){ // 0: Metric = cm (defalut) 1: Imperial = inch
  _eValue = (360.0/argEncoderValuePer360Rotation) * (3.141592 / 180.0);
  _wRadius = argWheelRadius;
  _wDistance = argDistanceBetweenTwoWheels;
  _unit = argDistanceUnit;
}

SimpleDeadReckoning::SimpleDeadReckoning(){   // defalut
  _eValue = (360.0/122.0) * (3.141592 / 180.0) ;    // click per 360/122 degree
  _wRadius = 0.6;  //  2*r = 6.78;
  _wDistance = 5.8;  // 15.5 cm between two wheels
  _unit = 0;        // Metric (cm) unit 
}

  
void SimpleDeadReckoning::updateLocation(float argLeftEncoder, float argRightEncoder, float argTheta){
   _pLeftOdom = _leftOdom ;
   _pRightOdom = _rightOdom;
   _pTheta = _theta;
   _pCheckTime = _checkTime;

   _leftOdom = argLeftEncoder ;
   _rightOdom = argRightEncoder;
   _checkTime = millis();

   float dt = (_checkTime - _pCheckTime) / 1000.0f;
   if (dt > 0) {
     float vL = (((_leftOdom - _pLeftOdom)*_eValue) / dt);
     float vR = (((_rightOdom - _pRightOdom )*_eValue) / dt);
     float wK = (vL - vR) / _wDistance;
     float deltaTheta = wK * dt;
     _theta += deltaTheta;
   }

   float vL = ((_leftOdom - _pLeftOdom)*_eValue) ;
   float vR = ((_rightOdom - _pRightOdom )*_eValue) ;
   float vK = (vL + vR) / 2;
   float deltaS = vK * _wRadius;

  
  _pXLocation = _xLocation;
  _pYLocation = _yLocation; 

// Euler integration
  _xLocation = _pXLocation + cos(_pTheta)*deltaS;
  _yLocation = _pYLocation + sin(_pTheta)*deltaS;
  
// Runge-Kutta integration
//  _xLocation = _pXLocation + (deltaS * cos( _pTheta + (deltaTheta/2.0) )) ;
//  _yLocation = _pYLocation + (deltaS * sin( _pTheta + (deltaTheta/2.0) )) ;


// Runge-Kutta integration (4th order)
// need to be implemented 



}



void SimpleDeadReckoning::updateOdometry(void){  
  _theta += 1 ;
}

float SimpleDeadReckoning::getTheta(void){
  return _theta;
}
float SimpleDeadReckoning::getLeftOdom(void){
  return _leftOdom;
}
float SimpleDeadReckoning::getRightOdom(void){
  return _rightOdom;
}
float SimpleDeadReckoning::getXLocation(void){
  return _xLocation;
}
float SimpleDeadReckoning::getYLocation(void){
  return _yLocation;
}
void SimpleDeadReckoning::setXLocation(float argX){
  _xLocation = argX ;
}
void SimpleDeadReckoning::setYLocation(float argY){
  _yLocation = argY;
}
long SimpleDeadReckoning::getCheckTime(void){
  return _checkTime;
}
long SimpleDeadReckoning::getLastCheckTime(void){
  return _pCheckTime;
}


