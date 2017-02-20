
#include "Motor.h"
#include "Arduino.h"


Motor::Motor(void (*forwardfn)(),void (*reversefn)(),void (*stopfn)(), int (*positionfn)(),void (*debugfn)(char *)){
  _forwardfn = forwardfn;
  _reversefn = reversefn;
  _stopfn = stopfn;
  _positionfn = positionfn;
  _debugfn = debugfn;
  _motorOn = 0;
  _checkTimeInterval = ROTORCHECKINTERVAL;
  _approachStopValue = APPROACHDEG;
  _minMoveStep = ROTORMINSTEPVAL;

}

void Motor::setCheckInterval(unsigned int v){
  _checkTimeInterval = v;
}

void Motor::setApproachStopValue(unsigned int v){
  _approachStopValue=v;
}

void Motor::setMinMoveStep(unsigned int v){
  _minMoveStep=v;
}

void Motor::stop(void){
  _stopfn();
  _motorOn=0;  
}


void Motor::debug(char *format, ...){
  if (_debugfn){
    static char buffer[64];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer, 64, format, args);
    va_end (args);
    _debugfn (buffer);
  }
}

void Motor::go (int target){
  int rotorPos;
  
  debug ("Go %d",target);

  rotorPos = _positionfn();
  
  if (abs(rotorPos-target)<MINDELTA){
    debug ("dsmall");
    return;
  }

  _targetPos=target;
  _delta2Target = abs(_targetPos-rotorPos);
  _rotorMoveStartAt = millis();
  _rotorCheckTime = _rotorMoveStartAt+_checkTimeInterval;

  if (_targetPos>rotorPos){
    _motorOn=1;
    _forwardfn();
  } else{
    _reversefn();
    _motorOn=-1;
  }
}

void Motor::sm(){
  unsigned long millisnow;
  int d2t; // delta 2 target
  int d2t1;
  int rotorPos;
  rotorPos = _positionfn();
  if (_motorOn!=0){
    // rotor motor is running !
    millisnow= millis();

    
    d2t = abs(rotorPos-_targetPos);
    // allow negative values for approach
    if (_motorOn==1){ d2t1 = _targetPos-rotorPos;} else {d2t1 = rotorPos-_targetPos;}    

    debug ("%d - %d - %d",rotorPos,d2t,d2t1);   

    // check if motor approach target
    if (d2t1 < _approachStopValue){
      _motorOn=0;
      _stopfn();      
    }
    // check for motor movement
    if (millisnow>_rotorCheckTime){
      _rotorCheckTime = millisnow+_checkTimeInterval;

      if (d2t>_delta2Target){
        _motorOn=0;
        _stopfn(); 
        debug ("bad dir");   
        // motor turns in wrong direction
      }
      if (abs(d2t-_delta2Target)<_minMoveStep){
        // motor does not turn or turn too slowly
        _motorOn=0;
        _stopfn ();      
        debug ("not moving");     
      }
      _delta2Target=d2t;
    }
  }
}






