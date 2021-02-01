/*
FILENAME...  MCBL2805Driver.h
USAGE...     Motor driver support for the Faulhaber MCBL2805 controller.

Michael Sintschuk
December 22, 2020

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// No controller-specific parameters yet
#define NUM_MCBL2805_PARAMS 0  

class epicsShareClass MCBL2805Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MCBL2805Axis(class MCBL2805Controller *pC);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);

private:
  MCBL2805Controller *pC_;        /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  double currentPosition_;
  double highLimit_;
  double lowLimit_;
  
friend class MCBL2805Controller;
};

class epicsShareClass MCBL2805Controller : public asynMotorController {
public:
  MCBL2805Controller(const char *portName, const char *serialPortName, int controllerID, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  MCBL2805Axis* getAxis(asynUser *pasynUser);
  MCBL2805Axis* getAxis(int axisNo);
  asynStatus writeMCBL2805();
  asynStatus writeMCBL2805(const char *output, double timeout);

private:
  int controllerID_;
  char controllerVersion_[40];

  friend class MCBL2805Axis;
};
