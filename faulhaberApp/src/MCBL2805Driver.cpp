/*
FILENAME... MCBL2805Driver.cpp
USAGE...    Motor driver support for the Faulhaber MCBL2805 controller.

Michael Sintschuk
December 22, 2020

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "MCBL2805Driver.h"

#define MCBL2805_TIMEOUT 2.0
#define LINUX_WRITE_DELAY 0.1

/** Creates a new MCBL2805Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName    The name of the drvAsynSerialPort that was created previously to connect to the MCBL2805 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
MCBL2805Controller::MCBL2805Controller(const char *portName, const char *serialPortName, int controllerID, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, 1, NUM_MCBL2805_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
   controllerID_(controllerID)
                    
{
  asynStatus status;
  static const char *functionName = "MCBL2805Controller::MCBL2805Controller";

  /* Connect to MCBL2805 controller */
  status = pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to MCBL2805 controller\n",
      functionName);
    return;
  }
  
  // Flush any characters that controller has, read firmware version
  sprintf(outString_, "%dVER", controllerID_);
  status = writeReadController();
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot read version information from MCBL2805 controller\n",
      functionName);
    return;
  }
  strcpy(controllerVersion_, &inString_[4]);

  // Create the axis object
  new MCBL2805Axis(this);

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new MCBL2805Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName       The name of the drvAsynIPPPort that was created previously to connect to the MCBL2805 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" {

int MCBL2805CreateController(const char *portName, const char *serialPortName, int controllerID, 
                             int movingPollPeriod, int idlePollPeriod)
{
  new MCBL2805Controller(portName, serialPortName, controllerID, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

} // extern "C" 


/** Writes a string to the controller.
  * Calls writeMCBL2805() with a default location of the string to write and a default timeout. */ 
asynStatus MCBL2805Controller::writeMCBL2805()
{
  return writeMCBL2805(outString_, MCBL2805_TIMEOUT);
}

/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus MCBL2805Controller::writeMCBL2805(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeMCBL2805";
  
  status = pasynOctetSyncIO->write(pasynUserController_, output,
                                   strlen(output), timeout, &nwrite);
                                   
  // On Linux it seems to be necessary to delay a short time between writes
  #ifdef linux
  epicsThreadSleep(LINUX_WRITE_DELAY);
  #endif
                                  
  return status ;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void MCBL2805Controller::report(FILE *fp, int level)
{
  fprintf(fp, "MCBL2805 motor driver %s, controllerID=%d, version=\"%s\n"
              "  moving poll period=%f, idle poll period=%f\n", 
    this->portName, controllerID_, controllerVersion_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an MCBL2805Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
MCBL2805Axis* MCBL2805Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<MCBL2805Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an MCBL2805Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
MCBL2805Axis* MCBL2805Controller::getAxis(int axisNo)
{
  return static_cast<MCBL2805Axis*>(asynMotorController::getAxis(axisNo));
}


// These are the MCBL2805Axis methods

/** Creates a new MCBL2805Axis object.
  * \param[in] pC Pointer to the MCBL2805Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
MCBL2805Axis::MCBL2805Axis(MCBL2805Controller *pC)
  : asynMotorAxis(pC, 0),
    pC_(pC), 
    currentPosition_(0.)
{
  static const char *functionName = "MCBL2805Axis::MCBL2805Axis";

  // this probably needs adjustment!
  stepSize_ = 1000.;
  
  // Read the low and high software limits
  sprintf(pC_->outString_, "%dGNL", pC->controllerID_);
  pC_->writeReadController();
  lowLimit_ = atof(&pC_->inString_[3]);
  sprintf(pC_->outString_, "%dGPL", pC->controllerID_);
  pC_->writeReadController();
  highLimit_ = atof(&pC_->inString_[3]);

  // activate the motor
  sprintf(pC_->outString_, "%dEN", pC_->controllerID_);
  status = pC_->writeMCBL2805();  

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void MCBL2805Axis::report(FILE *fp, int level)
{
  if (level > 0) {

    fprintf(fp, "  currentPosition=%f\n"
                "  stepSize=%f, lowLimit=%f, highLimit=%f\n",
            currentPosition_, 
            stepSize_, lowLimit_, highLimit_,);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus MCBL2805Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "MCBL2805Axis::move";
  
  // Acceleration and velocity are saved in the EEPROM, we dont want to change them

  if (relative) {
    sprintf(pC_->outString_, "%dLR%f", pC_->controllerID_, position*stepSize_);
    status = pC_->writeMCBL2805();
  } else {
    sprintf(pC_->outString_, "%dLA%f", pC_->controllerID_, position*stepSize_);
    status = pC_->writeMCBL2805();
  }
  
  sprintf(pC_->outString_, "%dM", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  return status;
}

asynStatus MCBL2805Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  //static const char *functionName = "MCBL2805Axis::home";
  
  // Initialize homesequence
  sprintf(pC_->outString_, "%dGOHOSEQ", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // now we have to wait until the motor reaches the limit
  // to-do: wait for the velocity to be zero... possible?
  epicsThreadSleep(10.0);
  
  // motor runs out to Hall zero and sets the position value to 0
  sprintf(pC_->outString_, "%dGOHIX", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // to-do: wait for the velocity to be zero... possible?
  epicsThreadSleep(5.0);
  
  // Switches to encoder mode. Uses external encoder for actual position
  sprintf(pC_->outString_, "%dENCMOD", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // actual speed is given by the Hall Sensors
  sprintf(pC_->outString_, "%dHALLSPEED", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // move 10mm
  sprintf(pC_->outString_, "%dLA-10000", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  sprintf(pC_->outString_, "%dM", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // to-do: wait for the velocity to be zero... possible?
  epicsThreadSleep(5.0);
  
  // set this position to 1095mm
  sprintf(pC_->outString_, "%dHO1095000", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  // activate the limit switches, at an edge the motor will stop
  sprintf(pC_->outString_, "%dHL6", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  
  setIntegerParam(pC_->motorStatusHome_, 1);
  setIntegerParam(pC_->motorStatusHomed_, 1);
  
  return status;
}

asynStatus MCBL2805Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "MCBL2805Axis::stop";

  sprintf(pC_->outString_, "%dV0", pC_->controllerID_);
  status = pC_->writeMCBL2805();
  return status;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus MCBL2805Axis::poll(bool *moving)
{ 
  int done=1;
  double position;
  unsigned int status;
  unsigned int state;
  int highLimit=0, lowLimit=0;
  int count;
  asynStatus comStatus;
  
  // still TO-DO !!!

  // Read the current motor position
  sprintf(pC_->outString_, "%dPOS", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TPxxx"
  position = atof(&pC_->inString_[3]);
  currentPosition_ = position /stepSize_;
  setDoubleParam(pC_->motorPosition_, currentPosition_);

  // Read the moving status of this motor
  //Returns a status number: 
  //Motor on, motion not in progress 81 
  //Motor on, motion in progress 80 
  //Motor off, motion not in progress 64
  sprintf(pC_->outString_, "%dTS?", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  
  // The response string is of the form "1TSabcdef"
  count = sscanf(pC_->inString_, "%*dTS%*4c%x", &status);
  if (count != 1) goto skip;

  state = status & 0xff;
  if ((state == 0x1e) || (state == 0x28)) done = 0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // The meaning of the error bits is different for the CC, AGP, and PP
  if ((conexModel_ == ModelConexCC) || (conexModel_ == ModelConexPP)) {
    if (status & 0x100) lowLimit = 1;
    if (status & 0x200) highLimit = 1;
  }
  
  setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
  setIntegerParam(pC_->motorStatusHighLimit_, highLimit);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg MCBL2805CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg MCBL2805CreateControllerArg1 = {"Serial port name", iocshArgString};
static const iocshArg MCBL2805CreateControllerArg2 = {"Controller ID", iocshArgInt};
static const iocshArg MCBL2805CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg MCBL2805CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const MCBL2805CreateControllerArgs[] = {&MCBL2805CreateControllerArg0,
                                                                &MCBL2805CreateControllerArg1,
                                                                &MCBL2805CreateControllerArg2,
                                                                &MCBL2805CreateControllerArg3,
                                                                &MCBL2805CreateControllerArg4};
static const iocshFuncDef MCBL2805CreateControllerDef = {"MCBL2805CreateController", 5, MCBL2805CreateControllerArgs};
static void MCBL2805CreateContollerCallFunc(const iocshArgBuf *args)
{
  MCBL2805CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void MCBL2805Register(void)
{
  iocshRegister(&MCBL2805CreateControllerDef, MCBL2805CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(MCBL2805Register);
}
