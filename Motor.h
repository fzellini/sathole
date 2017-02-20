#ifndef motor_h
  #define motor_h

  #define MINDELTA 2                 // don't move if  moving delta less than this value
  #define ROTORCHECKINTERVAL 1000    // check for movement each msec specified , default value
  #define APPROACHDEG 2              // turn off motor if position delta is < than this value, default
  #define ROTORMINSTEPVAL 2          // if motor position is less than this value at interval, assume not moving, default value

  class Motor{
    public:
      // function to fw,rew, stop motor and get current position
      Motor(void (*forwardfn)(),void (*reversefn)(),void (*stopfn)(), int (*positionfn)(),void (*debugfn)(char *));
      // advance state machine
      void sm ();
      void go (int );
      void debug(char *, ...);
//      void debug(char *);
      void setCheckInterval(unsigned int);
      void setApproachStopValue(unsigned int);
      void setMinMoveStep(unsigned int);
      void stop(void);
      
    private:
      void (*_forwardfn)();
      void (*_reversefn)();      
      void (*_stopfn)();      
      int (*_positionfn)();      
      void (*_debugfn)(char *);
      unsigned char _motorOn; // 0=off, 1=cw, -1=ccw
      int _currentPosition;
      int _lastPosition;
      int _targetPos;
      int _delta2Target;
      unsigned long _rotorMoveStartAt;
      unsigned long _rotorCheckTime;
      unsigned int _checkTimeInterval;
      unsigned int _approachStopValue;
      unsigned int _minMoveStep;
  };

#endif
