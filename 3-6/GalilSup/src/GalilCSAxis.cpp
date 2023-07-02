//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// Licence as published by the Free Software Foundation; either
// version 2.1 of the Licence, or (at your option) any later version.
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public Licence for more details.
//
// You should have received a copy of the GNU Lesser General Public
// Licence along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
//
// Contact details:
// Mark Clift
// email: padmoz@tpg.com.au

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <algorithm> //std::unique, std::distance

using namespace std; //cout ostringstream vector string

#include <epicsString.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <errlog.h>

extern "C" {
#include "sCalcPostfix.h"
}

#include "postfix.h"

#include <asynOctetSyncIO.h>

#include "GalilController.h"
#include <epicsExport.h>

static void pollServicesThreadC(void *pPvt);
static void startDeferredMovesThreadC(void *pPvt);
static void eventMonitorThreadC(void *pPvt);

// These are the GalilCSAxis methods

/** Creates a new GalilCSAxis object.
  */
GalilCSAxis::GalilCSAxis(class GalilController *pC, 	//The GalilController
	     char axisname)
  : asynMotorAxis(pC, (toupper(axisname) - AASCII)),
    pC_(pC), pollRequest_(10, sizeof(int))
{
  unsigned i;
  //store axis details
  //Store axis name
  axisName_ = (char)(toupper(axisname));

  //List of real motors related to this CS motor as extracted from this CS Motor transform equation
  //Real motor equations are reverse kinematic equations
  revaxes_ = (char *)calloc(MAX_GALIL_AXES, sizeof(char));

  //List of CS motors related to this CS motor as extracted from the reverse kinematic equations
  //CS motor equations are forward kinematic equations
  fwdaxes_ = (char *)calloc(MAX_GALIL_AXES, sizeof(char));

  //forward kinematic transform used to calculate the CS motor position from real motor positions
  forward_ = (char *)calloc(MAX_GALIL_STRING_SIZE, sizeof(char));

  //Forward kinematic variables List of Q-Z
  fwdvars_ = (char *)calloc(MAX_GALIL_AXES, sizeof(char));

  //Forward kinematic substitutes List of A-P
  fwdsubs_ = (char *)calloc(MAX_GALIL_AXES, sizeof(char));

  //Reverse transforms for the real axis
  reverse_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char*));
  //Reverse transforms variables
  revvars_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char*));
  //Reverse transforms substitutes
  revsubs_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char*));

  for (i = 0; i < MAX_GALIL_AXES; i++)
     {
     //Reverse transforms for the real axis
     reverse_[i] = (char *)calloc(MAX_GALIL_STRING_SIZE, sizeof(char));
     //Reverse transforms variables
     revvars_[i] = (char *)calloc(MAX_GALIL_STRING_SIZE, sizeof(char));
     //Reverse transforms substitutes
     revsubs_[i] = (char *)calloc(MAX_GALIL_STRING_SIZE, sizeof(char));
     }

  //set defaults
  setDefaults();

  // Create the thread that will service poll requests
  // To write to the controller
  epicsThreadCreate("pollServices", 
                    epicsThreadPriorityMax,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)pollServicesThreadC, (void *)this);

  //Create begin motion event
  beginEvent_ = epicsEventMustCreate(epicsEventEmpty);
  //Create event monitor start event
  eventMonitorStart_ = epicsEventMustCreate(epicsEventEmpty);
  //Create event monitor done event
  eventMonitorDone_ = epicsEventMustCreate(epicsEventEmpty);

  // Create the event monitor thread
  epicsThreadCreate("eventMonitor", 
                    epicsThreadPriorityMax,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)eventMonitorThreadC, (void *)this);

  //Create start deferred moves event
  deferredMoveStart_ = epicsEventMustCreate(epicsEventEmpty);

  // Create the start deferred moves thread
  epicsThreadCreate("startDeferredMoves", 
                    epicsThreadPriorityMax,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)startDeferredMovesThreadC, (void *)this);
}

//GalilAxis destructor
GalilCSAxis::~GalilCSAxis()
{
   unsigned i;

   //Set flag axis shut down in progress
   shuttingDown_ = true;
   //Now flag set, send eventMonitorThread to shutdown
   epicsEventSignal(eventMonitorStart_);
   //Now flag set, send startDeferredMovesThread to shutdown
   epicsEventSignal(deferredMoveStart_);

   //Free the ram we used for kinematics
   free(revaxes_);
   free(fwdaxes_);
   free(forward_);
   free(fwdvars_);
   free(fwdsubs_);

   for (i = 0; i < MAX_GALIL_AXES; i++)
      {
      free(reverse_[i]);
      free(revvars_[i]);
      free(revsubs_[i]);
      }

   free(reverse_);
   free(revvars_);
   free(revsubs_);

  //Free RAM used
  if (profilePositions_ != NULL)
     free(profilePositions_);

  //Destroy events
  //Sleep to preempt this thread, and give time
  //for threads to exit
  epicsThreadSleep(.002);
  epicsEventDestroy(beginEvent_);
  epicsEventDestroy(eventMonitorStart_);
  epicsEventDestroy(eventMonitorDone_);
  epicsEventDestroy(deferredMoveStart_);
}

/*--------------------------------------------------------------------------------*/
/* Store settings, set defaults for motor */
/*--------------------------------------------------------------------------------*/

asynStatus GalilCSAxis::setDefaults(void)
{
  //const char *functionName = "GalilCSAxis::setDefaults";
  unsigned i;

  //Five polls considered minimum to detect move start, stop
  double multiplier = 5.0 / (BEGIN_TIMEOUT / (pC_->updatePeriod_ / 1000.0));
  //Min multiplier is 1
  multiplier = (multiplier > 1) ? multiplier : 1;

  //Store axis in ParamList
  setIntegerParam(pC_->GalilAxis_, axisNo_);
  //We assume motor with encoder
  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  //Default all reverse kinematics to null strings
  for (i = 0; i < MAX_GALIL_AXES; i++)
     pC_->setStringParam(axisNo_, pC_->GalilCSMotorReverseA_ + i, "");

  //Give default readback values for positions, movement direction
  motor_position_ = 0;
  last_motor_position_ = 0;
  encoder_position_ = 0;
  direction_ = 1;
  //Default setpoint
  setPoint_ = 0.0;
  //Pass default step count/aux encoder value to motorRecord
  setDoubleParam(pC_->motorPosition_, motor_position_);
  //Pass default encoder value to motorRecord
  setDoubleParam(pC_->motorEncoderPosition_, encoder_position_);
  //Pass default direction value to motorRecord
  setIntegerParam(pC_->motorStatusDirection_, direction_);
  //Motor not homing now
  //This flag does include JAH
  setIntegerParam(pC_->GalilHoming_, 0);
  //Driver is not requesting CSaxis stop
  stopInternal_ = false;
  //Driver has not stopped CSaxis motor record
  stoppedMR_ = false;
  //Default motor record write enable
  pC_->setIntegerParam(axisNo_, pC_->GalilMotorSetValEnable_, 0);
  //Default motor record stop
  pC_->setIntegerParam(axisNo_, pC_->GalilMotorRecordStop_, 0);
  //Motor stop mesg not sent to pollServices thread
  stopSent_ = false;
  //CSAxis has not started a move
  move_started_ = false;
  //CSAxis move is not a jog move
  moveVelocity_ = false;
  //Kinematics haven't been altered
  kinematicsAltered_ = false;
  //Axis not ready until necessary motor record fields have been pushed into driver
  lastaxisReady_ = axisReady_ = false;
  //This csaxis is not performing a deferred move
  deferredMove_ = false;
  //Shutdown flag
  shuttingDown_ = false;
  //Tell poller we dont want signal when events occur
  requestedEventSent_ = true;
  //Default event timeout
  requestedTimeout_ = BEGIN_TIMEOUT * multiplier;
  return asynSuccess;
}

/** Check requested motor velocities
  * \param[in] npos - After kinematics these are the requested motor positions Units=Steps
  * \param[in] nvel - After kinematics these are the requested  motor velocities Units=Steps/s
  * \param[in] naccel - After kinematics these are the requested motor accelerations Units=Steps/s/s */
asynStatus GalilCSAxis::checkMotorVelocities(double npos[], double nvel[], double naccel[])
{
  unsigned i;				//Looping
  double incmove[MAX_GALIL_AXES];	//Real motor relative move distances
  GalilAxis *pAxis;			//Real motor instance
  char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg
  double mres;				//Motor record mres
  double mpos, epos;			//Motor position, encoder position for real motor
  double vectorVelocity = 0;		//Computed vector velocity
  double vectorDistance = 0;		//Computed vector distance
  double vectorAcceleration = 0;	//Computed vector acceleration
  double aDistance;			//Computed acceleration distance
  double aCorrection;			//Correction when vector shorter than acceleration distance
  double vel;				//Temp variable used to calculate real motor velocity
  double vmax;				//Motor record vmax
  int status;

  if (deferredMode_)
     {
     //Sync start and stop uses linear mode
     //Use vector mathematics to check requested motor velocities
     //Loop thru all real motors, calculate vector distance and velocity
     for (i = 0; revaxes_[i] != '\0'; i++)
        {
        //Retrieve the axis instance
        pAxis = pC_->getAxis(revaxes_[i] - AASCII);
        //Process or skip
        if (!pAxis) continue;
        //Get the readbacks for the axis
        status = pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorEncoderPosition_, &epos);
        status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorPosition_, &mpos);
        //Calculate incremental move distance in steps
        //Here we must use the register the controller uses for positioning
        //This may differ from the real axis motor record readback as set by ueip
        incmove[i] = (pAxis->ctrlUseMain_) ? npos[i] - epos : npos[i] - mpos;
        //Sum vector distance, velocity for non zero move increments
        if (fabs(incmove[i]) != 0.0)
           {
           //Calculate vector distance
           vectorDistance += pow(incmove[i], 2);
           //Calculate vector velocity
           vectorVelocity += pow(nvel[i], 2);
           //Calculate vector acceleration
           vectorAcceleration += pow(naccel[i], 2);
           }
        }

     //Calculate vector distance
     vectorDistance = sqrt(vectorDistance);
     if (vectorDistance > 0)
        {
        //Calculate vector velocity
        vectorVelocity = sqrt(vectorVelocity);
        //Calculate vector acceleration
        vectorAcceleration = sqrt(vectorAcceleration);
        //Compensate for cases where vector is shorter than acceleration distance
        aDistance = 0.5 * vectorAcceleration * pow(vectorVelocity/vectorAcceleration, 2);
        aCorrection = (vectorDistance/aDistance > 1) ? 1 : vectorDistance/aDistance;
        //Calculate actual motor speeds as controller does in linear mode
        for (i = 0; revaxes_[i] != '\0'; i++)
           {
           //Retrieve needed motor record fields
           pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilMotorVmax_, &vmax);
           pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
           //Calculate this motors actual velocity in egu
           vel = (incmove[i]/vectorDistance) * vectorVelocity * mres * aCorrection;
           //Is motor velocity within allowed limit vmax
           if (fabs(trunc(vel * 1000000.0)) > fabs(trunc(vmax * 1000000.0)))
              {
              sprintf(mesg, "Move failed, axis %c velocity %2.6lf > VMAX %2.6lf", revaxes_[i], fabs(vel), fabs(vmax));
              pC_->setCtrlError(mesg);
              return asynError;
              }
           }
        }//vectorDistance > 0
    }
  else
    {
    //Sync start only mode
    //Loop thru all real motors and check velocity
    for (i = 0; revaxes_[i] != '\0'; i++)
       {
       //Retrieve needed motor record fields
       pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilMotorVmax_, &vmax);
       pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
       //Calculate requested velocity in egu
       vel = nvel[i] * mres;
       //Is motor velocity within allowed limit vmax
       if (fabs(trunc(vel * 1000000.0)) > fabs(trunc(vmax * 1000000.0)))
           {
           sprintf(mesg, "Move failed, axis %c velocity %2.6lf > VMAX %2.6lf", revaxes_[i], fabs(vel), fabs(vmax));
           pC_->setCtrlError(mesg);
           return asynError;
           }
       }
    }

  return asynSuccess;
}

//Check csaxis axes are good to go
asynStatus GalilCSAxis::beginCheck(const char *caller)
{
   GalilAxis *pAxis;
   unsigned i;
   //Check reverse axes are ready to go
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      if (!pAxis) continue;
      //Check axis is ready to go, supply arbitary velocity
      //Dont reset controller message
      if (pAxis->beginCheck(caller, axisName_, 100, false))
         return asynError;
      }
   //Return value
   return asynSuccess;
}

//Check axis motor record settings
//Return error if record is not ready for new moves
asynStatus GalilCSAxis::checkMRSettings(const char *caller)
{
   GalilAxis *pAxis;		//Reverse axis
   unsigned i;			//Looping
   string mesg = "";		//Controller mesg
   int spmg;			//Motor record spmg field
   int set;			//Motor record set field
   int status;			//Return status

   //Check reverse axis motor record (MR) settings
   for (i = 0; revaxes_[i] != '\0'; i++) {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      if (!pAxis) continue;
      //Check motor record settings
      if (pAxis->checkMRSettings(caller, axisName_, moveVelocity_))
         return asynError; //Return if any error
   }

   //Check this CSAxis MR settings
   //Retrieve needed params
   status = pC_->getIntegerParam(axisNo_, pC_->GalilStopPauseMoveGo_, &spmg);
   status |= pC_->getIntegerParam(axisNo_, pC_->GalilMotorSet_, &set);
   //Return if any paramlist error
   if (status) return asynError;

   //Check motor record status
   if (spmg != 3 && spmg != 2) {
      mesg = string(caller) + " " + string(1, axisName_) + " failed, " + string(1, axisName_);
      mesg += " spmg is not set to \"go\" or \"move\"";
   }
   if (set && !moveVelocity_) {
      mesg = string(caller) + " " + string(1, axisName_) + " failed, " + string(1, axisName_);
      mesg += " set field is not set to \"use\"";
   }

   //Display any controller mesg
   if (!mesg.empty()) {
      pC_->setCtrlError(mesg);
      status = asynError;
   }

   //Return status
   return asynStatus(status);
}

//Enforce CSAxis completion order
int GalilCSAxis::enforceCSAxisCompletionOrder(int csmoving)
{
   GalilCSAxis *pCSAxis;	//Galil CSAxis
   int fmoving;			//Related CSAxis moving status
   int fdmov;			//Related CSAxis done status
   unsigned i;			//Looping

   //Enforce CSAxis completion order
   //Completion order is axis order within fwdaxes array
   if (axisName_ != fwdaxes_[0] && move_started_)
      {
      for (i = 0; fwdaxes_[i] != '\0'; i++)
         {
         //Retrieve the CSAxis
         pCSAxis = pC_->getCSAxis(fwdaxes_[i] - AASCII);
         //Skip or process
         if (!pCSAxis) continue;
         //Don't apply completion rules to self
         if (axisName_ != pCSAxis->axisName_)
            {
            //Retrieve moving status
            pC_->getIntegerParam(pCSAxis->axisNo_, pC_->motorStatusMoving_, &fmoving);
            //Retrieve dmov status.  Related CSAxis may be doing backlash or retry
            pC_->getIntegerParam(pCSAxis->axisNo_, pC_->GalilDmov_, &fdmov);
            //Or related CSAxis moving status
            csmoving |= ((!fdmov) || (fmoving));
            }
         }
      }

   //Return result
   return csmoving;
}

/** Check axis physical limits given move request
  * \param[in] caller - Caller function name
  * \param[in] npos - Reverse (real) axis new positions Units=steps */
asynStatus GalilCSAxis::checkLimits(const char *caller, double npos[])
{
   GalilAxis *pAxis;		//GalilAxis instance
   unsigned i;      		//Looping
   int status = asynSuccess;	//Return status

   //Loop thru reverse axis
   //check physical limits given move request
   for (i = 0; revaxes_[i] != '\0'; i++) {
      //Retrieve axis instance
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      //Skip or continue
      if (!pAxis) continue;
      //Check axis physical limits
      if (pAxis->checkLimits(caller, axisName_, npos[i]))
         return asynError; //Return if any error
   }

   //Return success
   return (asynStatus)status;
}

/** Check axis soft limits given move request
  * \param[in] caller - Caller function name
  * \param[in] npos - Reverse (real) axis new positions Units=steps */
asynStatus GalilCSAxis::checkSoftLimits(const char *caller, double npos[])
{
   GalilAxis *pAxis;		//GalilAxis instance
   unsigned i;      		//Looping
   int status = asynSuccess;	//Return status

   //Check soft limits for move, not jog
   if (moveVelocity_)
      return asynSuccess;

   //Loop thru reverse axis
   //check physical limits given move request
   for (i = 0; revaxes_[i] != '\0'; i++) {
      //Retrieve axis instance
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      //Skip or continue
      if (!pAxis) continue;
      //Check axis physical limits
      if (pAxis->checkSoftLimits(caller, axisName_, npos[i]))
         return asynError; //Return if any error
   }

   //Return success
   return (asynStatus)status;
}


/** Check all axis settings
  * \param[in] caller - Caller function name
  * \param[in] npos - Reverse (real) axis new positions Units=steps
  * \param[in] nvel - Reverse (real) axis new velocities Units=steps/Sec
  * \param[in] naccel - Reverse (real) axis new accelerations Units=steps/Sec2
  * \param[in] bCheck - Should beginCheck be done ? */
asynStatus GalilCSAxis::checkAllSettings(const char *caller, double npos[], double nvel[], double naccel[], bool bCheck) {
   int status;                      //Return code

   //Check requested motor velocities
   status = checkMotorVelocities(npos, nvel, naccel);
   //Check motor record settings
   status |= checkMRSettings(caller);
   //Check limits given move request
   status |= checkLimits(caller, npos);
   //Check soft limits given move request, not for jog
   if (!moveVelocity_)
      status |= checkSoftLimits(caller, npos);
   //Check csaxis axes are good to go
   if (bCheck)
      status |= beginCheck(caller);

   return asynStatus(status);
   //Return result
}

//Monitor CSAxis move, stop if problem
//Called by poller
asynStatus GalilCSAxis::monitorCSAxisMove(void)
{
   GalilAxis *pAxis;	//Reverse axis
   int dmov;		//Dmov status
   int ssc;		//Reverse axis stop code that caused the CSAxis stop
   int sc;		//Stop code of remaining reverse axis
   unsigned i;		//Looping

   //Check reverse axis move status
   //Stop CSAxis if problem
   if (!done_ && move_started_ && !deferredMove_ && !stopInternal_) {
      for (i = 0; revaxes_[i] != '\0'; i++) {
         //Retrieve reverse axis instance
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Skip or continue
         if (!pAxis) continue;
         //Set axis stop code
         ssc = pAxis->stop_code_;
         //Set flag if an axis stops
         if ((ssc == MOTOR_STOP_FWD) || (ssc == MOTOR_STOP_REV) ||
            (ssc == MOTOR_STOP_ONERR) ||
            (ssc == MOTOR_STOP_ENC) || (ssc == MOTOR_STOP_AMP) ||
            (ssc == MOTOR_STOP_ECATCOMM) || (ssc == MOTOR_STOP_ECATAMP)) {
            stopInternal_ = true;
            break;
         }
      }//For
   }

   //Retrieve dmov status
   pC_->getIntegerParam(axisNo_, pC_->GalilDmov_, &dmov);
   //Clear stop csaxis flags if dmov true
   if (stopInternal_ && dmov && last_done_ && done_) {
      stopInternal_ = false;
      stopSent_ = false;
      stop_reason_ = MOTOR_OKAY;
   }

   //Stop CSAxis if requested
   if (!done_ && !deferredMove_ && stopInternal_ && move_started_ && !stopSent_) {
      //Stop the real motors in the CSAxis
      for (i = 0; revaxes_[i] != '\0'; i++) {
         //Retrieve the axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;
         //Push this axis stop code into convenience variable
         sc = pAxis->stop_code_;
         //Stop the CSAxis, if we find a revaxis is not stopping already
         if (!pAxis->done_ &&
          sc != MOTOR_STOP_FWD && sc != MOTOR_STOP_REV && sc != MOTOR_STOP_STOP &&
          sc != MOTOR_STOP_ONERR && sc != MOTOR_STOP_ENC && sc != MOTOR_STOP_AMP &&
          sc != MOTOR_STOP_ECATCOMM && sc != MOTOR_STOP_ECATAMP) {
            //Set CSAxis stop reason
            stop_reason_ = ssc;
            //Tell CSAxis to stop
            pollRequest_.send((void*)&MOTOR_STOP, sizeof(int));
            //Flag CSAxis stop message sent
            stopSent_ = true;
         }
      }//For
   }

   return asynSuccess;
}

//Clear CSAxis move dynamics at move completion
//Called by poller
asynStatus GalilCSAxis::clearCSAxisDynamics(void)
{
   GalilAxis *pAxis;	//Reverse axis
   unsigned i;		//Looping

   if (move_started_ && done_) {
      //CSAxis move completed
      move_started_ = false;
      //After move clear axis useCSA flag
      for (i = 0; revaxes_[i] != '\0'; i++) {
         //Retrieve the axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;
         //Clear useCSADynamics flag
         pAxis->useCSADynamics_ = false;
      }
   }

   return asynSuccess;
}

//Setup CSAxis move flags
asynStatus GalilCSAxis::setupCSAxisMove(bool moveVelocity)
{
   //Set deferred move flag, causing move status for this csaxis to become true
   deferredMove_ = true;
   
   //Now CSAxis moving status true
   //Set flag that move started
   move_started_ = true;

   //Set moveVelocity if instructed now that CSAxis moving status true
   if (moveVelocity)
      moveVelocity_ = true;

   return asynSuccess;
}

/* C Function which runs the pollServices thread */ 
static void pollServicesThreadC(void *pPvt)
{
  GalilCSAxis *pC = (GalilCSAxis*)pPvt;
  pC->pollServices();
}

//Service slow and infrequent requests from poll thread to write to the controller
//We do this in a separate thread so the poll thread is not slowed, and poll thread doesnt have a lock
void GalilCSAxis::pollServices(void)
{
  int request;	//Request number

  while (true)
     {
     //Wait for poll to request a service
     pollRequest_.receive(&request, sizeof(int));
     //Obtain the lock
     pC_->lock();
     //What did poll request
     switch (request)
        {
        case MOTOR_STOP: stopInternal();
                         break;
        default: break;
        }
     //Release the lock
     pC_->unlock();
     }
}

//C Function which runs the start deferred moves thread
//Used to start CSAxis moves when movesDeferred is set false, not for jog
static void startDeferredMovesThreadC(void *pPvt)
{
  GalilCSAxis *pC = (GalilCSAxis*)pPvt;
  pC->startDeferredMovesThread();
}

//Start deferred moves runs in it's own thread
//Used to start CSAxis moves automatically
//when movesDeferred is set false, not for jog
void GalilCSAxis::startDeferredMovesThread() {
  string mesg;			//Controller message
  unsigned i, j;		//Looping
  GalilAxis *pAxis;		//Reverse axes
  string axes;			//List of revaxes that received a new setpoint

  while (true) {
    //Wait for request
    epicsEventWait(deferredMoveStart_);
    //Check for shutdown request
    if (shuttingDown_)
       return; //Exit the thread

    //Obtain lock
    pC_->lock();
    //Clear the axes list
    axes.clear();

    //Construct list of axis that received new setpoint
    for (i = 0; revaxes_[i] != '\0'; i++) {
       //Retrieve axis instance
       pAxis = pC_->getAxis(revaxes_[i] - AASCII);
       //Skip or process
       if (!pAxis) continue;
       //Check if revaxis received a new setpoint from this CSAxis
       if (pAxis->useCSADynamics_)
          axes += revaxes_[i];
    }//For
    
    //Wait till GalilAxis::move has been called for all revaxis that received a new setpoint
    for (string::iterator it = axes.begin(); it != axes.end(); it++) {
       //Retrieve axis instance
       pAxis = pC_->getAxis(*it - AASCII);
       //Skip or process
       if (!pAxis) continue;
       //Zero while loop cycle count
       j = 0;
       //Unlock mutex so GalilAxis::move is called
       //Also give chance for sync poller to get the lock
       pC_->unlock();
       //Wait till GalilAxis::move has been called for this axis
       while (!pAxis->deferredMove_) {
          //Preempt this thread
          epicsThreadSleep(.001);
          //Give up after 200ms/preempts
          if (j++ > 200)
             break;
       }//While
       //Check for error
       if (!pAxis->deferredMove_ && pAxis->moveThruRecord_) {
          //GalilAxis::move wasn't called
          //Set flag false
          pAxis->moveThruRecord_ = false;
          //Disable further writes to axis motor record from driver
          pC_->setIntegerParam(pAxis->axisNo_, pC_->GalilMotorSetValEnable_, 0);
       }
       //Done, move on to next motor
       pC_->lock();
    }//For

    //Start CSAxis move
    //Clear deferredMove flag on all reverse axis
    pC_->setDeferredMoves(false);

    //Release lock
    pC_->unlock();

  }// While(true)
}

/* C Function which runs the event monitor thread */ 
static void eventMonitorThreadC(void *pPvt) {
  GalilCSAxis *pC = (GalilCSAxis*)pPvt;
  pC->eventMonitorThread();
}

//Event monitor runs in its own thread
//Used to synchronize threads to events sent by poller
void GalilCSAxis::eventMonitorThread() {
  while (true) {
    //Wait for request
    epicsEventWait(eventMonitorStart_);
    //Check for shutdown request
    if (shuttingDown_)
       return; //Exit the thread
    //Wait for requested event
    eventResult_ = epicsEventWaitWithTimeout(requestedEvent_, requestedTimeout_);
    if (eventResult_ != epicsEventWaitOK) {
       //Timeout, or error occurred
       //Tell poller we dont want signal when events occur
       requestedEventSent_ = true;
    }

    //Signal waiting thread begin monitor done
    epicsEventSignal(eventMonitorDone_);
  }
}

/** Send axis events to waiting threads
  * Called by poller without lock after passing
  * motor status to motor record
  * \param[in] inmotion Moving status that ignores CSAxis deferredMove*/
void GalilCSAxis::sendAxisEvents(bool inmotion) {

   //Check for work
   if (requestedEventSent_) return; //Nothing to do

   //Motion begin event = confirmed moving status true delivered to MR
   if (requestedEvent_ == beginEvent_)
      if (inmotion) {
         //Reverse axis real motion is reason CSAxis move status true
         //Begin event
         epicsEventSignal(beginEvent_);
         //Tell poller we dont want signal when events occur
         requestedEventSent_ = true;
      }
}

/** Move the motor to an absolute location or by a relative amount.
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
  * by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus GalilCSAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  string functionName = "GalilCSAxis::move";
  int i;				//Looping, indexing
  double npos[MAX_GALIL_AXES];		//Real axis position targets
  double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
  double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
  int lastaxis = -1;			//Last axis in move when moves not deferred
  GalilAxis *pAxis;			//Pointer to GalilAxis instance
  int deferredMode;			//Deferred move mode
  double readback;			//CSAxis readback
  int ueip;				//CSAxis use encoder if present
  double abspos; 			//Move absolute position requested
  int status;				//Return status
  bool anyAxisSucceed = false;          //Track setting of revaxes setpoint when moves deferred

  //Determine CSAxis readback, and absolute position requested Units=steps
  status = pC_->getIntegerParam(axisNo_, pC_->GalilUseEncoder_, &ueip);
  if (!status) {
     //Calculate CSAxis readback in steps
     readback = (ueip) ? encoder_position_ : motor_position_;
     //Determine absolute move request position
     abspos = (relative) ? readback + position : position;
  }
  else
     return asynError;

  //Store CSAxis setpoint for use in kinematics
  if (!moveVelocity_)
     setPoint_ = abspos;

  //Clear controller message
  pC_->setCtrlError("");

  //Retrieve deferred moves mode
  pC_->getIntegerParam(pC_->GalilDeferredMode_, &deferredMode);
  deferredMode_ = deferredMode;

  //Are moves to be deferred ?
  if (pC_->movesDeferred_ != 0) {
     //Moves are deferred
     //Store parameters for deferred move in GalilCSAxis
     deferredPosition_ = position;
     pC_->getIntegerParam(0, pC_->GalilCoordSys_, &deferredCoordsys_);
     deferredVelocity_ = maxVelocity;
     deferredAcceleration_ = acceleration;
     deferredRelative_ = relative;

     //Perform reverse transform and get new axis (real) motor positions, velocities
     status = reverseTransform(abspos, maxVelocity, acceleration, npos, nvel, naccel);

     //Check all motor settings
     status |= checkAllSettings(functionName.c_str(), npos, nvel, naccel, false);

     //Write the motor setpoints, but dont move
     if (!status) {
        //Write the motor setpoints
        for (i = 0; revaxes_[i] != '\0'; i++) {
           //Retrieve the axis
           pAxis = pC_->getAxis(revaxes_[i] - AASCII);
           if (!pAxis) continue;
           //Write motor set point, but dont start motion
           //Moves are sent to real axis motor records
           //Backlash, and retries are supported
           if (!pAxis->moveThruMotorRecord(npos[i])) {
              //Requested move equal or larger than 1 motor step, move success
              //GalilAxis::move will be called when GalilCSAxis::move exits
              //Tell this real axis to use CSAxis dynamics
              pAxis->setCSADynamics(naccel[i], nvel[i]);
              anyAxisSucceed = true;
           }
        }
        if (anyAxisSucceed) {
           //Set CSAxis move flags
           setupCSAxisMove(false);
        }
     }
  }
  else {
     //Moves are not deferred
     //Perform reverse transform and get new motor positions
     status = reverseTransform(abspos, maxVelocity, acceleration, npos, nvel, naccel);
     //Check all motor settings
     status |= checkAllSettings(functionName.c_str(), npos, nvel, naccel);

     //Select a free coordinate system
     if (deferredMode && !status)
        if (selectFreeCoordinateSystem() == -1)
           status = asynError;

     if (!status) {
        //Do the move, using deferredMoves facility in GalilController
        //Set controller deferred move flag
        pC_->setDeferredMoves(true);
        //Write the motor setpoints
        for (i = 0; revaxes_[i] != '\0'; i++) {
           //Retrieve the axis
           pAxis = pC_->getAxis(revaxes_[i] - AASCII);
           if (!pAxis) continue;
           //Write motor set point, but dont start motion
           if (moveVelocity_) {
              //Tell this real axis to use CSAxis dynamics
              pAxis->setCSADynamics(naccel[i], nvel[i]);
              //Jog moves are accomplished through internal calls to GalilAxis move
              pAxis->move(npos[i], relative, minVelocity, nvel[i], naccel[i]);
           }
           else {
              //Moves are sent to real axis motor records
              //Backlash, and retries are supported
              if (!pAxis->moveThruMotorRecord(npos[i])) {
                 //Requested move equal or larger than 1 motor step, move success
                 //GalilAxis::move will be called when GalilCSAxis::move exits
                 //Tell this real axis to use CSAxis dynamics
                 pAxis->setCSADynamics(naccel[i], nvel[i]);
                 //Track last axis with successful move
                 lastaxis = i;
              }
           }
        }

        //For moves, not for jog
        //Setup method to start deferred moves
        if (!moveVelocity_ && lastaxis != -1) {
            //At least 1 axis pushed a new setpoint to mr
            //Set CSAxis move flags
            setupCSAxisMove(false);
            //Signal separate thread to release deferred moves
            //When GalilCSAxis::move exits
            epicsEventSignal(deferredMoveStart_);
        }

        //For jog
        //Also for move when requested position < 1 motor step from readback
        //Must start deferred moves here
        if (moveVelocity_ || !deferredMove_)
           pC_->setDeferredMoves(false);
     }//Status

     //Clear deferred move flag for failed jog
     //This can occur when move fails initial checks (eg. checkMotorVelocities)
     if (moveVelocity_ && status) {
        //Clear deferred move flag
        deferredMove_ = false;
        //Move did not start
        move_started_ = false;
        moveVelocity_ = false;
     }
  }//Moves not deferred

  //Return status
  return asynSuccess;
}

/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus GalilCSAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  //static const char *functionName = "GalilCSAxis::moveVelocity";
  double position;
  int deferredMode;			//Deferred move mode
  double npos[MAX_GALIL_AXES];		//Real axis position targets
  double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
  double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
  unsigned i;				//Looping
  double max;				//Max increment or absolute position that the controller supports
  double increment;			//Increment or absolute position value to try on coordinate system axis
  double axisIncrement;			//Individual axis increment or absolute position
  double readback;			//Reverse axis readback
  bool done = false;			//Found valid move values
  bool problem = false;			//Move increment or absolute position too large
  GalilAxis *pAxis;			//Pointer to GalilAxis instance
  int status = asynSuccess;

  //Retrieve deferred moves mode
  pC_->getIntegerParam(pC_->GalilDeferredMode_, &deferredMode);
  //Set max
  //In sync start stop mode this is the maximum incremental distance
  //In sync start only mode this is the maximum absolute position
  max = (deferredMode) ? MAX_GALIL_LINEAR_INCREMENT : MAX_GALIL_ABSOLUTE_MOVE/1.1;
  //Set initial attempt
  increment = max;
  //Reduce move size until within controller limitations
  while (!done && !status)
     {
     problem = false;
     //Give increment correct sign given desired move direction
     //Sync start only move "increment" is an absolute position
     position = (maxVelocity > 0) ? increment : -increment;
     if (deferredMode)
        {
        //Sync start stop move is an increment
        //Convert relative move to absolute position
        position = motor_position_ + position;
        }
     //Perform reverse transform and get new motor positions
     status = reverseTransform(position, maxVelocity, acceleration, npos, nvel, naccel);
     if (!status)
        {
        //Sanity check new motor positions
        for (i = 0; revaxes_[i] != '\0'; i++)
           {
           //Retrieve the axis
           pAxis = pC_->getAxis(revaxes_[i] - AASCII);
           if (!pAxis) continue;
           if (deferredMode)
              {
              //Sync start stop move
              //Calculate this motors readback
              readback = (pAxis->ctrlUseMain_) ? pAxis->encoder_position_ : pAxis->motor_position_;
              //Convert absolute position back to incremental distance for sanity check
              axisIncrement = npos[i] - readback;
              }
           else //Store sync start only absolute position for sanity check
              axisIncrement = npos[i];
           //Sanity check absolute position, or incrmental value
           if (ceil(fabs(axisIncrement)) > max)
              {
              //Absolute position, or increment is beyond controller max
              //Reduce the value by 10%
              increment = increment/1.1;
              //Flag we need to try again
              problem = true;
              }
           }//Sanity checking

        if (!problem) //Success
           done = true;
        }//Status
     }//while

  if (!pC_->movesDeferred_ && !status)
     {
     //Moves are not deferred, so do the jog
     //This CSAxis has a valid deferred move
     //Set CSAxis move flags
     setupCSAxisMove(true);
     //Do the "jog"
     move(position, 0, minVelocity, fabs(maxVelocity), acceleration);
     }

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Stop the motor.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus GalilCSAxis::stop(double acceleration)
{
  unsigned i;		//Looping
  GalilAxis *pAxis;	//GalilAxis

  //Check source of call to GalilCSAxis::stop
  if (stoppedMR_) {
     //GalilCSAxis:stop called this time by driver calling stopMotorRecord
     //Reset flag now GalilCSAxis:stop has been called
     stoppedMR_ = false;
     //Set CSAxis motor record stop to zero
     pC_->setIntegerParam(axisNo_, pC_->GalilMotorRecordStop_, 0);
     //Backlash, retries are now prevented
     //Do nothing further, let internal stop continue undisturbed
     return asynSuccess;
  }

  //Prepare to stop
  for (i = 0; revaxes_[i] != '\0'; i++) {
     //Retrieve the axis
     pAxis = pC_->getAxis(revaxes_[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //Check if axis homing
     if (pAxis->homing_) {
        //Cancel any home operation
        sprintf(pC_->cmd_, "home%c=0", axisName_);
        pC_->sync_writeReadController();
        //Cancel limit/home switch jog off operations that may be underway
        //Set deceleration back to normal
        sprintf(pC_->cmd_, "hjog%c=0;DC%c=nrmdc%c", axisName_, axisName_, axisName_);
        pC_->sync_writeReadController();
        //Set homing flag false
        //This flag does not include JAH
        pAxis->homing_ = false;
        //This flag does include JAH
        pC_->setIntegerParam(pAxis->axisNo_, pC_->GalilHoming_, 0);
     }
     //Set flag so backlash, retries from this axis motor record are prevented
     pAxis->stopInternal_ = true;
     //For internal stop, prevent axis motor record issuing backlash, retries
     pAxis->stopMotorRecord();
  }

  //For internal stop, prevent CSAxis motor record issuing backlash, retries
  stopMotorRecord();

  //Issue stop
  if (strcmp(revaxes_, "") != 0) {
     //revaxes_ cannot be empty, else all threads on controller get killed
     //Stop the real motors that this CSAxis started
     sprintf(pC_->cmd_, "ST %s", revaxes_);
     pC_->sync_writeReadController();
  }

  //Clear defer move flag
  deferredMove_ = false;

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Stop the motor.  Called by driver internally
  * Blocks backlash, retries attempts from motorRecord until dmov
  * \param[in] emergencyStop - Emergency stop true or false */
asynStatus GalilCSAxis::stopInternal(bool emergencyStop)
{
   double acceleration[MAX_GALIL_AXES];
   double accel;
   string cmd;
   GalilAxis *pAxis;
   unsigned i;

   //Indicate stop source is the driver not motor record
   stopInternal_ = true;
   
   //Default acceleration array
   for (i = 0; i < MAX_GALIL_AXES; i++)
      acceleration[i] = -1;

   //Loop thru real motor list, and determine correct deceleration to use for each motor
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      //Process or skip
      if (!pAxis) continue;
      //Determine correct deceleration
      acceleration[revaxes_[i] - AASCII] = (emergencyStop) ? pAxis->limdc_ : pAxis->csaAcceleration_;
      }

   //Issue stop with arbitary deceleration (its ignored)
   stop(100);
   
   //Change the deceleration of the motors if required
   //Empty command string
   cmd = "DC ";
   //Construct deceleration command string
   for (i = 0; i < MAX_GALIL_AXES; i++)
      {
      if (acceleration[i] != -1)
         {
         //Add acceleration specified to command string
         accel = (long)lrint(acceleration[i]/1024.0) * 1024;
         cmd += tsp(accel, 0) + ',';
         }
      else //Add comma separator as required
         cmd += ',';
      }
   //Copy command into buffer used by sync_writeReadController
   strcpy(pC_->cmd_, cmd.c_str());
   //Change deceleration for all revaxes in single command
   pC_->sync_writeReadController();

   //Always return success. Dont need more error mesgs
   return asynSuccess;
}

/** Stop CSaxis motor record.  Called by driver internally.
  * Prevents backlash, and retry attempts from motorRecord */
asynStatus GalilCSAxis::stopMotorRecord(void) {
   int status = asynSuccess; //Return status
   //What is the source of the stop request ?
   //Possible sources are the driver, or the motor record
   if (stopInternal_) {
      //Set flag indicating next call to GalilCSAxis::stop caused by
      //setting axis motor record stop field 1
      stoppedMR_ = true;
      //Stop request is from driver, not motor record
      //Send stop to this axis MR to prevent backlash, and retry attempts
      status = pC_->setIntegerParam(axisNo_, pC_->GalilMotorRecordStop_, 1);
      //Do callbacks
      pC_->callParamCallbacks(axisNo_);
   }
   //Return result
   return (asynStatus)status;
}

/** Move the motors to the home position.  Attempt simultaneous start (useSwitch=true)
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus GalilCSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
   static const char *functionName = "GalilCSAxis::home";
   unsigned i;				//Looping
   unsigned j = 0, k = 0;		//Build axes lists
   GalilAxis *pAxis;			//GalilAxis
   double npos[MAX_GALIL_AXES];		//Real axis position targets
   double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
   double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
   char maxes[MAX_GALIL_AXES];		//List of axis that we move home
   char paxes[MAX_GALIL_AXES];		//List of axis that we prepare to move home
   int dir;				//Reverse axis home direction
   int hometypeallowed;			//Home type allowed
   int useSwitch;			//Use switch when homing
   int ssiinput;			//SSI encoder register
   int ssicapable;			//SSI capable
   int ssiconnect;			//SSI connected status
   int status;				//Driver status
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg

   //Clear controller message
   pC_->setCtrlError("");

   //Default list of axes we move home
   strcpy(maxes, "");
   //Default list of axes we prepare to move home
   strcpy(paxes, "");

   //Perform reverse transform to get real motor accelerations and velocities
   //We dont care about position here (arbitary 100)
   status = reverseTransform(100, maxVelocity, acceleration, npos, nvel, naccel);
   //Check csaxis axes are good to go
   status |= beginCheck(functionName);

   if (!status)
      {
      //Loop thru real motor list, and send them home
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
         //Retrieve axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;

         //Retrieve home type allowed for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHomeAllowed_, &hometypeallowed);
         //Retrieve use switch for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilUseSwitch_, &useSwitch);
         //Retrieve SSI encoder config
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilSSIInput_, &ssiinput);
         pC_->getIntegerParam(pC_->GalilSSICapable_, &ssicapable);
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilSSIConnected_, &ssiconnect);

         //Work out direction to home this axis
         //CSAxis home is forward
         if (forwards)
            {
            if (hometypeallowed == HOME_FWD || hometypeallowed == HOME_BOTH)
               dir = 1;  //Axis allows forward home
            else
               {
               //Axis does not allow forward home, so try reverse instead
               if (hometypeallowed == HOME_REV)
                  dir = 0;
               }
            }
         else
            {
            //CSAxis home is reverse
            if (hometypeallowed == HOME_REV || hometypeallowed == HOME_BOTH)
               dir = 0;  //Axis allows reverse home
            else
               {
               //Axis does not allow reverse home, so try forward instead
               if (hometypeallowed == HOME_FWD)
                  dir = 1;
               }
            }
         //Set homing parameters for direction we worked out above
         if (hometypeallowed)
            {
            if (ssiinput && ssicapable && ssiconnect)
               {
               //Nothing to do
               sprintf(mesg, "%c axis has no home process because of SSI encoder", pAxis->axisName_);
               //Set controller error mesg monitor
               pC_->setCtrlError(mesg);
               }
            else
               {
               //set acceleration, velocity is ignored here
               pAxis->setAccelVelocity(naccel[i], maxVelocity);
               //Set home velocity, direction
               pAxis->setupHome(nvel[i], dir);
               //Create two lists of axes
               if (useSwitch)
                  {
                  //Move axes.  List we jog toward switch
                  maxes[j++] = revaxes_[i];
                  maxes[j] = '\0';
                  }
               else
                  {
                  //Prepare axes.  List we prepare for move
                  paxes[k++] = revaxes_[i];
                  paxes[k] = '\0';
                  }
               }
            }
         else
            {//This reverse axis does not allow homing at all
            sprintf(mesg, "%c axis extra settings do not allow homing", pAxis->axisName_);
            pC_->setCtrlError(mesg);
            }
         }//for loop

      //Start motors simultanously
      pC_->beginGroupMotion(maxes, paxes);
      //Create list containing both move, and prepare to move motors
      strcat(maxes, paxes);
      
      //Loop thru the axes lists, and set home flags
      //Which starts home program on controller
      for (i = 0; maxes[i] != '\0'; i++)
         {
         //Retrieve axis
         pAxis = pC_->getAxis(maxes[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;
         //Retrieve home type allowed for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHomeAllowed_, &hometypeallowed);
         //Retrieve use switch for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilUseSwitch_, &useSwitch);
         //If revaxes allows homing, set flags now home jog has started
         if (hometypeallowed)
            {
            if (!useSwitch)
               {
               //Because we are calling galil code before motion begins (useSwitch=false)
               //Reset stopped time, so homing doesn't timeout immediately
               pAxis->resetStoppedTime_ = true;  //Request poll thread reset stopped time if done
               //Wait for poller to reset stopped time on this axis
               //ensure synchronous poller is not blocked
               pC_->unlock();
               epicsEventWaitWithTimeout(pAxis->stoppedTimeReset_, pC_->updatePeriod_/1000.0);
               pC_->lock();
               }
            //Start was successful
            //This homing status does not include JAH
            //Flag homing now true
            pAxis->homing_ = true;
            //This homing status does include JAH
            //Flag homing now true
            pC_->setIntegerParam(pAxis->axisNo_, pC_->GalilHoming_, 1);
            //Homing has not been cancelled yet
            pAxis->cancelHomeSent_ = false;
            //tell controller which axis we are doing a home on
            //We do this last so home algorithm doesn't cancel home jog in incase motor
            //is sitting on opposite limit to which we are homing
            sprintf(pC_->cmd_, "home%c=1", pAxis->axisName_);
            pC_->sync_writeReadController();
            }
         }
      }

   return (asynStatus)status;
}

/** Select a free coordinate system, or return -1
  */
int GalilCSAxis::selectFreeCoordinateSystem(void)
{
  int coordsys;
  int profileExecuteStatus;
  int moving;
  int current_coordsys;
  int status;

  //Get profile execute status
  pC_->getIntegerParam(0, pC_->profileExecuteState_, &profileExecuteStatus);
  //Return error if profileExecuteStatus not done
  if (profileExecuteStatus != PROFILE_EXECUTE_DONE)
	return -1;
  //Retrieve current coordinate system
  pC_->getIntegerParam(0, pC_->GalilCoordSys_, &current_coordsys);
  //Check if current coordinate system free
  pC_->getIntegerParam(current_coordsys, pC_->GalilCoordSysMoving_, &moving);
  coordsys = current_coordsys;
  if (moving)
	{
	//Wasnt free.  Try the other coordsys
	coordsys = (current_coordsys) ? 0 : 1;
	//Check if coordinate system free
  	pC_->getIntegerParam(coordsys, pC_->GalilCoordSysMoving_, &moving);
	if (moving)
		return -1;	//Wasnt free
	else
		{
		//Other coordinate system was free, use it
		sprintf(pC_->cmd_, "CA %c", (coordsys == 0) ? 'S' : 'T');
		//Write setting to controller
		status = pC_->sync_writeReadController();
		//Proceed if coordsys change ok
		if (status)
			return -1;
		//Set change in coordsys in paramList
		pC_->setIntegerParam(0, pC_->GalilCoordSys_, coordsys);
		}
	}

  return coordsys;
}

/** Contruct axis list from given transform equation
  * \param[in] equation - Kinematic transform equation
  * \param[out] axes - List of axes found in the transform equation
  */
asynStatus GalilCSAxis::obtainAxisList(char axis, char *equation, char *axes)
{
   unsigned i;				//Looping
   size_t found;			//Used to search axes string
   string axes_s = "";			//std::string axes list will be constructed
   bool forward = (axis >= 'I' && axis <='P') ? true : false;	//Transform direction forward or reverse
   char first = (forward) ? 'I' : 'A';	//Axis range that is not allowed
   char last = (forward) ? 'P' : 'H';	//Axis range that is not allowed
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg
  
   //Construct list of axis found in equation
   for (i = 0; equation[i] != '\0'; i++)
      {
      equation[i] = toupper(equation[i]);
      //Create list of axis found in equation
      if ((equation[i] >= 'A' && equation[i] <= 'P' && !isalnum(equation[i+1]) && !i) ||
         (i && equation[i] >= 'A' && equation[i] <= 'P' && !isalnum(equation[i+1]) && !isalnum(equation[i-1])))
         {
         //Ensure no CS motors in forward equations, and no real motors in reverse equations
         if ((equation[i] >= first && equation[i] <= last && !isalnum(equation[i+1]) && !i) ||
             (i && equation[i] >= first && equation[i] <= last && !isalnum(equation[i+1]) && !isalnum(equation[i-1])))
            {
            if (axisReady_)
               {
               //CS motors were found in forward transform, or real motors found in reverse transform
               //reverseTransform and forwardTransform methods will not function in this case
               sprintf(mesg, "%c transform substition failed", axis);
               pC_->setCtrlError(mesg);
               return asynError;
               }
            }

         //Use std::string to avoid adding axis multiple times
         found = axes_s.find(equation[i]);
         //Add axis to list if hasnt been added already
         if (found == string::npos)
            {
            //Store the axis in the list
            axes_s.push_back(equation[i]);
            }
         }//Axis list
      }
   //Sort the axes list
   sort(axes_s.begin(), axes_s.end());
   //Remove any duplicates
   axes_s.erase(std::unique(axes_s.begin(), axes_s.end()), axes_s.end());
   //Copy into axes
   strcpy(axes, axes_s.c_str());

   return asynSuccess;
}

/** For reverseTransform and forwardTransform methods to work properly
  * Reverse transform equations must only contain CS motors, and 
  * Forward transform equations must only contain Real motors.
  * Perform string substitution on provided equation to satisfy above rules
  * \param[in] axis    	    - Axis that the provided kinematic equation relates to (ie. axis=equation)
  * \param[in/out] equation - Kinematic transform equation
  */
asynStatus GalilCSAxis::substituteTransforms(char axis, char *equation)
{
  unsigned i;							//Looping
  bool forward = (axis >= 'I' && axis <='P') ? true : false;	//Transform direction forward or reverse
  string equation_s = equation;					//For string substitution
  char subst_transform[MAX_GALIL_STRING_SIZE];			//The substitute transform		
  char first = (forward) ? 'I' : 'A';				//Axis we substitute for complete transform
  char last = (forward) ? 'P' : 'H';				//Axis we substitute for complete transform
  char mesg[MAX_GALIL_STRING_SIZE];				//Controller error mesg

  //Scan through transform equation
  for (i = 0; equation[i] != '\0'; i++)
     {
     equation[i] = toupper(equation[i]);
     if ((equation[i] >= first && equation[i] <= last && !isalnum(equation[i+1]) && !i) ||
         (i && equation[i] >= first && equation[i] <= last && !isalnum(equation[i+1]) && !isalnum(equation[i-1])))
        {
        //Found a motor we want to substitute with a complete transform
        //Retrieve the substitute transform
        if (forward)
           pC_->getStringParam(equation[i] - AASCII, pC_->GalilCSMotorForward_ , MAX_GALIL_STRING_SIZE, subst_transform);
        else
           pC_->getStringParam(axisNo_, pC_->GalilCSMotorReverseA_ + equation[i] - AASCII, MAX_GALIL_STRING_SIZE, subst_transform);

        //Substitute motor letter with complete transform
        equation_s.replace(i, i+1, subst_transform);
        if (equation_s.length() < MAX_GALIL_STRING_SIZE - 1)
           strcpy(equation, equation_s.c_str());
        else
           {
           sprintf(mesg, "%c transform, including substitutes, is > 256 bytes", axis);
           pC_->setCtrlError(mesg);
           return asynError;
           }
        }
     }

  return asynSuccess;
}

/** Find kinematic variables Q-X and substitute them for variable in range A-P for sCalcPerform
  * \param[in] axis    	    - Axis that provided kinematic equation relates to (ie. axis=equation)
  * \param[in] equation     - Kinematic transform equation
  * \param[in] axes   	    - List of axis found in the equation
  * \param[out] variables   - List variables found in the equation
  * \param[out] substitutes - List substitutes that replaced the variables
  */
asynStatus GalilCSAxis::substituteVariables(char axis, char *equation, char *axes, char *vars, char *subs)
{
   unsigned arg_status[SCALCARGS];	//sCalcPerform argument used status
   unsigned i, j, k;			//Looping/indexing
   bool findarg;			//Do we need to find an arg position for this variable, or have we done so already
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg

   //Default variable list
   strcpy(vars, "");

   //Default substitution list
   strcpy(subs, "");

   //Default sCalcPerform arg status.  No args used
   for (i = 0; i < SCALCARGS; i++)
      arg_status[i] = false;

   //Flag arg for motors in axes list as used
   for (i = 0; i < strlen(axes); i++)
      arg_status[axes[i] - AASCII] = true;

   //Flag arg for specified axis as used
   arg_status[axis - AASCII] = true;

   k = 0;
   //Now find variables in range Q-Z, and substitute them for variable in range A-P
   for (i = 0; i < strlen(equation); i++)
      {
      //Variable substitution
      if ((equation[i] >= 'Q' && equation[i] <= 'Z' && !isalpha(equation[i+1]) && !i) ||
         (i && equation[i] >= 'Q' && equation[i] <= 'Z' && !isalpha(equation[i+1]) && !isalpha(equation[i-1])))
         {
         findarg = true;
         //Check if we have found an arg position for this variable already
         for (j = 0; j < strlen(vars); j++)
            if (vars[j] == equation[i])
               {
               findarg = false;
               break;
               }

         if (findarg)
            {
            //We need to find an argument position
            //Store the found variable
            vars[k] = equation[i];
            vars[k + 1] = '\0';
            //Find free argument position
            for (j = 0; j < SCALCARGS; j++)
               {
               if (!arg_status[j])
                  {
                  //Found free arg position, use it
                  arg_status[j] = true;
                  //Calculate letter A-P which corresponds to this arg position
                  subs[k] = j + AASCII;
                  subs[k+1] = '\0';
                  break;
                  }
		//Did we find a free arg position for this kinematic variable ?
		if (j == SCALCARGS)
		   {
                   sprintf(mesg, "%c simplify kinematic equation", axis);
                   pC_->setCtrlError(mesg);
                   return asynError;
                   }
                }
             //Substitute the variable
             equation[i] = subs[k];
             k++;
             }
         else
             {
             //Substitute the variable
             equation[i] = subs[k];
             }
         }//Variable substition
      }

   return asynSuccess;
}

/** Parse a kinematic transform equation.  Used to parse kinematic equation into GalilCSAxis instance
  * \param[in] axis    	    - Axis that provided kinematic equation relates to (ie. axis=equation)
  * \param[in/out] equation - Kinematic transform equation
  * \param[out] axes   	    - List of axis found in the equation
  * \param[out] variables   - List variables found in the equation
  * \param[out] substitutes - List substitutes that replaced the variables
  */
asynStatus GalilCSAxis::parseTransform(char axis, char *equation, char *axes, char *vars, char *subs)
{  
   int status;

   //Remove spaces from provided equation
   string equation_s = equation;
   equation_s.erase (std::remove(equation_s.begin(), equation_s.end(), ' '), equation_s.end());
   strcpy(equation, equation_s.c_str());
   //Only CS motors appear in reverse equations
   //Only Real motors appear in forward equations
   //Apply substitution to ensure this is true or fail
   status = substituteTransforms(axis, equation);
   //Create axes list from transform equation
   status |= obtainAxisList(axis, equation, axes);
   //Kinematic variables no longer used.  Left for reference
   //Replace variables in range Q-X with variables in range A-P for sCalcPerform
   //status |= substituteVariables(axis, equation, axes, vars, subs);
  
   return asynStatus(status);
}

//Parse the kinematic equations and load into GalilCSAxis instance
asynStatus GalilCSAxis::parseTransforms(void)
{
  unsigned i;			//Looping
  int status;			//Return status
  string fwdaxes_s = "";	//Forward axes list
  GalilAxis *pAxis;		//GalilAxis

  //Flag kinematic error not yet reported
  kinematic_error_reported_ = false;

  //Retrieve forward kinematic equation for this cs axis (eg. I=(A+B)/2)
  status = pC_->getStringParam(axisNo_, pC_->GalilCSMotorForward_ , MAX_GALIL_STRING_SIZE, forward_);
  //Parse forward transform into GalilCSAxis instance
  status |= parseTransform(axisName_, forward_, revaxes_, fwdvars_, fwdsubs_);
 
  //Retrieve reverse kinematic equations for axes found in forward transform retrieved above
  for (i = 0; revaxes_[i] != '\0'; i++) {
     //Retrieve reverse transform for axis specified in revaxes_
     status |= pC_->getStringParam(axisNo_, pC_->GalilCSMotorReverseA_ + revaxes_[i] - AASCII, MAX_GALIL_STRING_SIZE, reverse_[i]);
     //Parse reverse transform into GalilCSAxis instance
     status |= parseTransform(revaxes_[i], reverse_[i], fwdaxes_, revvars_[i], revsubs_[i]);
     //Concat axis list found in last reverse transform
     fwdaxes_s += fwdaxes_;
     //Sort the axes list
     sort(fwdaxes_s.begin(), fwdaxes_s.end());
     //Remove any duplicates
     fwdaxes_s.erase(std::unique(fwdaxes_s.begin(), fwdaxes_s.end()), fwdaxes_s.end());
  }

  //Finalise fwdaxes_ list for this CSAxis
  strcpy(fwdaxes_, fwdaxes_s.c_str());

  //Loop thru reverse axis
  //Populate reverse axes "related CSAxis" list
  for (i = 0; revaxes_[i] != '\0'; i++) {
     //Retrieve axis instance
     pAxis = pC_->getAxis(revaxes_[i] - AASCII);
     //Skip or continue
     if (!pAxis) continue;
     //Populate reverse axes related CSAxis list
     strcpy(pAxis->csaxesList_, fwdaxes_);
  }

  return asynStatus(status);
}

/* Calculate limit orientation of each reverse axis in this CSAxis
 * \param[in] axes - Axis list
*/
asynStatus GalilCSAxis::calcAxisLimitOrientation(void)
{
   GalilAxis *pAxis;		//Motor axis instance
   unsigned i, j;		//Looping
   double largs[SCALCARGS];	//Limit arguments
   double orientation;		//How real axis limits are oriented relative to this CSAxis
   double first, second;	//Used to calculate how this CSAxis RBV changes as real axis RBV grows
                                //This determines limit orientation of real axis limits relative to this CSAxis
   int status = asynSuccess;    //Return status

   if (kinematicsAltered_)
      {
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
         //Set limit argument array to all 1's
         for (j = 0; j < SCALCARGS; j++)
            largs[j] = 1;
         //Retrieve the revaxis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         if (!pAxis) continue;
         //Insert 10 (unlikely result will be 0) for this revaxis
         largs[revaxes_[i] - AASCII] = 10;
         //Perform reverse transform
         status |= doCalc(forward_, largs, &first);
         //Insert 20 for this revaxis (ie. move revaxes forward) 
         largs[revaxes_[i] - AASCII] = 20;
         //Perform reverse transform
         status |= doCalc(forward_, largs, &second);
         //Orientation depends on how CSAxis changes as revaxes moves forward
         orientation = (second > first) ? 1 : -1;
         //Set limits orientation
         limitOrientation_[i] = (orientation > 0) ? consistent : not_consistent;
         }
      kinematicsAltered_ = false;
      }

  return (asynStatus)status;
}

/* Retrieve setpoints for forward axes (Related CSAxis) and pack into spargs
 * \param[out] spargs - Motor setpoint arguments dial Units=EGU
*/
asynStatus GalilCSAxis::packSetPointArgs(double spargs[])
{
  unsigned i;			//Looping
  GalilCSAxis *pCSAxis;		//GalilCSAxis instance
  double mres;			//Motor record mres
  int status = asynSuccess;	//Return status

  //Retrieve setpoints for all forward axes and pack into spargs
  //equation provided by user
  for (i = 0; fwdaxes_[i] != '\0'; i++)
     {
     //Retrieve CSAxis
     pCSAxis = pC_->getCSAxis(fwdaxes_[i] - AASCII);
     //Skip or continue
     if (!pCSAxis) continue;
     //Retrieve CSAxis mres
     status |= pC_->getDoubleParam(fwdaxes_[i] - AASCII, pC_->motorResolution_, &mres);
     //OFF, and DIR always 0 for CSAxis
     //Store CSAxis setpoint in user coordinates
     spargs[fwdaxes_[i] - AASCII] = pCSAxis->setPoint_ * mres;
     }
  return (asynStatus)status;
}

/* Retrieve readbacks for provided axes list, and pack into mrargs
 * \param[out] mrargs - Motor readback arguments Units=EGU
*/
asynStatus GalilCSAxis::packReadbackArgs(char *axes, double mrargs[])
{
  unsigned i;			//Looping
  double mpos, epos;		//Motor, and encoder readback data
  double mres, eres;		//Motor record mres, eres
  double off;			//Motor record off
  int dir, dirm;		//Motor record dir, direction multiplier
  int ueip;			//Use encoder if present
  int axisNo;			//Axis param list number
  int status = asynSuccess;

  //Retrieve readbacks for all axis and pack into mrargs
  //equation provided by user
  for (i = 0; axes[i] != '\0'; i++)
     {
     axisNo = axes[i] - AASCII;
     //Get the readbacks for the axis
     status |= pC_->getDoubleParam(axisNo, pC_->motorEncoderPosition_, &epos);
     status |= pC_->getDoubleParam(axisNo, pC_->motorPosition_, &mpos);
     //Retrieve needed motor record fields
     status |= pC_->getDoubleParam(axisNo, pC_->motorResolution_, &mres);
     status |= pC_->getDoubleParam(axisNo, pC_->GalilEncoderResolution_, &eres);
     status |= pC_->getIntegerParam(axisNo, pC_->GalilUseEncoder_, &ueip);
     //Note DIR and OFF always 0 for CSAxis (forward axes)
     status |= pC_->getIntegerParam(axisNo, pC_->GalilDirection_, &dir);
     status |= pC_->getDoubleParam(axisNo, pC_->GalilUserOffset_, &off);
     //Calculate direction multiplier
     dirm = (dir == 0) ? 1 : -1;
     //Pack motor readbacks for calc in user coordinates
     //Here we use position register for the real axis motor record as set by user via ueip field
     //This may be different from the register the controller uses for positioning
     if (!status)
        mrargs[axisNo] = (ueip) ? (epos * eres * dirm) + off : (mpos * mres * dirm) + off;
     }
  return (asynStatus)status;
}

/* Perform reverse kinematic transform for this coordinate system axis
 * And calculate real motor position, velocity and acceleration
 * \param[in] pos - CSAxis position required Units=Steps
 * \param[in] vel - CSAxis velocity required Units=Steps/s
 * \param[in] accel - CSAxis acceleration required Units/s/s
 * \param[out] npos - Calculated motor positions for the real axis Units=Steps
 * \param[out] nvel - Calculated motor velocities for the real axis Units=steps/s
 * \param[out] naccel - Calculated motor accelerations for the real axis Units-Steps/s/s
 * \param[in] useCSSetpoints - Use CSAxis setpoints in transform if true, else use readbacks
 * \param[in] profileMsg - Should error messages be shown as profile build message*/
asynStatus GalilCSAxis::reverseTransform(double pos, double vel, double accel, double npos[], double nvel[], double naccel[], bool useCSSetpoints, bool profileMsg)
{
  GalilAxis *pAxis;             //Reverse axis
  double mres;			//Motor record mres
  double eres;			//Motor record eres
  double vmax;                  //Motor record vmax
  double off;			//Motor record offset
  int dir, dirm;		//Motor record dir, and direction multiplier
  double ctargs[SCALCARGS] = {0}; //Coordinate transform arguments
  char mesg[MAX_GALIL_STRING_SIZE];//Controller error mesg
  double lowest_accel = 99999999;
  long lowest_accelhw;
  double accel_ratio;
  unsigned i;
  int ueip;
  int status = asynSuccess;
  double readback; // Used for both CSAxis readback and revaxis readback

  double csPosition; // Requested CSAxis position in user cooordinates Unit=EGU
  double csVelocity; // Requested CSAxis velocity in Unit=EGU/Second
  double csAcceleration; // Requested CSAxis acceleration in Unit=EGU/Second2
  double csTravelDistance; // Requested CSAxis incremental travel distance Unit=EGU
  double csTravelTime; // Requested CSAxis travel time assuming constant velocity Unit=Second
  double csAccelerationTime; // Requested CSAxis acceleration time Unit=Second
  double axisTravelDistance; // Requested reverse axis incremental travel distance Unit=EGU

  // Pack forward axes setpoints (ie. related CSAxis)
  if (useCSSetpoints)
     status |= packSetPointArgs(ctargs);
  else// Use forward axes readbacks instead
     status |= packReadbackArgs(fwdaxes_, ctargs);

  // Retrieve needed motor record parameters for this CSAxis
  status |= pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
  status |= pC_->getIntegerParam(axisNo_, pC_->GalilUseEncoder_, &ueip);
  if ((status |= pC_->getDoubleParam(axisNo_, pC_->GalilEncoderResolution_, \
     &eres)) != asynSuccess) {
     // Return if error retrieving parameters
     return (asynStatus)status;
  }

  // Calculate CSAxis readback in user coordinates
  // OFF, and DIR always 0 for csaxis
  readback = (ueip) ? encoder_position_ * eres : motor_position_ * mres;
 
  // Convert new CSAxis position, velocity, acceleration into user coordinates
  // OFF, and DIR always 0 for csaxis
  csPosition = pos * mres;
  csVelocity = fabs(vel * mres);
  csAcceleration = fabs(accel * mres);

  // Calculate CSAxis travel distance in user coordinates
  csTravelDistance = fabs(csPosition - readback);
  // Calculate CSAxis travel time assuming constant velocity Unit=Second
  csTravelTime = csTravelDistance / csVelocity;
  // Calculate CSAxis acceleration time Unit=Second
  csAccelerationTime = csVelocity / csAcceleration;

  // Substitute new motor position received for this CSAxis
  // Convert new position from steps into user coordinates
  // OFF, and DIR always 0 for csaxis
  ctargs[axisNo_] = csPosition;

  for (i = 0; revaxes_[i] != '\0'; i++) {
    // Retrieve the axis
    pAxis = pC_->getAxis(revaxes_[i] - AASCII);
    // Skip axis if null
    if (!pAxis) continue;
    // Check for empty reverse expression
    if (reverse_[i][0] == '\0') {
       sprintf(mesg, "reverseTransform fail: %c has empty reverse equation for axis %c", axisName_, revaxes_[i]);
       if (profileMsg)// Profile build message
          pC_->setStringParam(pC_->profileBuildMessage_, mesg);
       else// Axis related message (eg. home, move, jog, etc)
          pC_->setCtrlError(mesg);
       return asynError;
    }

    // Retrieve needed motor record parameters
    status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilEncoderResolution_, &eres);
    status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
    status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilUserOffset_, &off);
    status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilDirection_, &dir);
    status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilMotorVmax_, &vmax);

    if (!status) {
       // Calculate direction multiplier
       dirm = (dir == 0) ? 1 : -1;
       // Determine reverse axis readback in dial coordinates
       readback = (pAxis->ctrlUseMain_) ? (pAxis->encoder_position_ * eres) : (pAxis->motor_position_ * mres);
       // Convert readback to user coordinates
       readback = (readback * dirm) + off;

       // Perform reverse coordinate transform to derive the required real axis positions 
       // given new csaxis position
       status |= doCalc(reverse_[i], ctargs, &npos[i]);
       // Calculate revaxis travel distance
       axisTravelDistance = npos[i] - readback;
       // Set reverse axis travel time equal to CSAxis travel time
       // Then calculate required revaxis velocity
       nvel[i] = fabs(axisTravelDistance / csTravelTime);
       // Cap reverse axis velocity at motor record vmax
       nvel[i] = (nvel[i] > vmax) ? vmax : nvel[i];
       // Set reverse axis acceleration time equal to CSAxis acceleration time
       // Then calculate required revaxis acceleration
       naccel[i] = nvel[i] / csAccelerationTime;

       // Convert position from user coordinates to steps for move
       npos[i] = (npos[i] - off) / (mres * dirm);
       //Convert velocity from egu to steps for move
       nvel[i] = fabs(nvel[i] / mres);
       //Convert acceleration from egu to steps for move
       naccel[i] = fabs(naccel[i] / mres);
       // Find lowest acceleration in the group
       if ((trunc(naccel[i] * 1000000.0) != 0.000000) && (naccel[i] < lowest_accel)) {
          lowest_accel = naccel[i];
       }
    }//Status
  }//For
 
  //Refactor acceleration given limited acceleration resolution on controller
  if (!status) {
     //Find closest hardware setting for lowest acceleration found
     lowest_accelhw = (long)lrint(lowest_accel/1024.0) * 1024;
     lowest_accelhw = (lowest_accelhw == 0) ? 1024 : lowest_accelhw;
     //Now refactor real motor acceleration
     for (i = 0; revaxes_[i] != '\0'; i++) {
        accel_ratio = naccel[i]/lowest_accel;
        if (accel_ratio != 1.000000) {
          naccel[i] = lowest_accelhw * accel_ratio;
        }

     }//For
  }//Status

  return (asynStatus)status;
}

//Transform CSAxis profile into Axis profiles
asynStatus GalilCSAxis::transformCSAxisProfile(void)
{
  unsigned i, j;		//Looping
  GalilAxis *pAxis[MAX_GALIL_AXES];//GalilAxis
  double npos[MAX_GALIL_AXES];	//Real axis position targets
  double nvel[MAX_GALIL_AXES];	//Real axis velocity targets.  Ignored here, but needed
  double naccel[MAX_GALIL_AXES];//Real axis acceleration targets.  Ignored here, but needed
  double mres;			//Motor record motor resolution
  double off;			//Motor record offset
  int dir, dirm;		//Motor record direction
  int status;			//Return status
  int useCSAxis;		//useAxis flag for CSAxis
  int nPoints;			//Number of points in profile
  vector<double> calculatedPositions; //Calculated profiles of real axis

  //Retrieve required attributes from ParamList
  pC_->getIntegerParam(0, pC_->profileNumPoints_, &nPoints);
  pC_->getIntegerParam(axisNo_, pC_->profileUseAxis_, &useCSAxis);
  //Process or skip this CSAxis
  if (!useCSAxis)
     return asynSuccess;

  //Test transform equations before we get started
  if (reverseTransform(profilePositions_[0], 100, 100, npos, nvel, naccel, false, true))
     return asynError;

  //Retrieve the revaxes (real motors) in this CSAxis
  for (i = 0; revaxes_[i] != '\0'; i++)
     {
     pAxis[i] = pC_->getAxis(revaxes_[i] - AASCII);
     if (!pAxis[i]) //Return error if any GalilAxis not instantiated
        return asynError;
     else
        {
        //Transform will proceed, and later restore is required for this GalilAxis
        pAxis[i]->restoreProfile_ = true;
        //Allocate room to backup loaded profile for this real axis
        if (pAxis[i]->profileBackupPositions_)    free(pAxis[i]->profileBackupPositions_);
        pAxis[i]->profileBackupPositions_ =      (double *)calloc(pC_->maxProfilePoints_, sizeof(double));
        }
     }

   //Transform this CSAxis, and create Axis profile data
   for (i = 0; i < (unsigned)nPoints; i++)
      {
      //Perform reverse transform and get new axis (real) motor positions
      //We dont care about velocity (arbitary 100) and acceleration (arbitary 100) here
      reverseTransform(profilePositions_[i], 100, 100, npos, nvel, naccel, false, true);
      //Copy new axis profile data point into revaxes GalilAxis instances
      for (j = 0; revaxes_[j] != '\0'; j++)
           {
           //Backup GalilAxis profile data
           pAxis[j]->profileBackupPositions_[i] = pAxis[j]->profilePositions_[i];
           //Copy in new GalilAxis profile data from transform
           pAxis[j]->profilePositions_[i] = npos[j];
           }
      }//Transform complete

   //Upload new points to database readback waveforms
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve needed motor record fields
      status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
      status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilDirection_, &dir);
      status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilUserOffset_, &off);
      //Calculate direction multiplier
      dirm = (dir == 0) ? 1 : -1;
      //Clear the buffer
      calculatedPositions.clear();
      //Copy profile into calculatedPositions array
      for (j = 0; j < (unsigned)nPoints; j++)
         calculatedPositions.push_back((pAxis[i]->profilePositions_[j] * mres * dirm) + off);
      //Update calculatedPositions record for this revaxis (real motor)
      pC_->doCallbacksFloat64Array(&calculatedPositions[0], nPoints, pC_->GalilProfileCalculated_, revaxes_[i] - AASCII);
      }

  //All ok
  return asynSuccess;
}

//Perform forward kinematic transform using readback data, variables and store results in GalilCSAxis as csaxis readback
asynStatus GalilCSAxis::forwardTransform(void)
{
  double mrargs[SCALCARGS];	//Motor position readback args in egu used in the forward transform
  double position;		//CSAxis readback position
  double mres, eres;		//Motor record mres, and eres
  int status = asynSuccess;	//Asyn paramList return code

  //Pack position readback args for reverse axes (ie. real motor readbacks)
  status |= packReadbackArgs(revaxes_, mrargs);
  //Calculate real axis limit orientation
  calcAxisLimitOrientation();

  //Perform forward kinematic transforms
  if (!status) {
     //Retrieve needed motor record fields
     status |= pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
     status |= pC_->getDoubleParam(axisNo_, pC_->GalilEncoderResolution_, &eres);
     //Do the forward transform
     if (!status) {
        //Perform forward coordinate transform
        status |= doCalc(forward_, mrargs, &position);
        //Coordinate transform results
        //CSAxis OFF, and DIR always 0
        //CSAxis dial, and user coordinates are the same
        //Convert from user coordinates to steps for interaction with motor record
        encoder_position_ = position / eres;
        //Convert from user coordinates to steps for interaction with motor record
        motor_position_ = position / mres;
     }
  }

  return (asynStatus)status;
}

//Perform kinematic calculations
//Evaluate expression, return result
asynStatus GalilCSAxis::doCalc(const char *expr, double args[], double *result) {
   
   unsigned char rpn[512];		//Expression is converted to reverse polish notation 
   short err;
   bool error = false;			//Error status
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg
   int precision = 12;			//Hard code precision for now
    
   *result = 0.0;

   //For empty expressions
   if (expr[0] == '\0')
      return asynSuccess;

   //We use sCalcPostfix and sCalcPerform because it can handle upto 16 args
   if (sCalcPostfix(expr, rpn, &err))
      error = true;
   else if (sCalcPerform(args, SCALCARGS, NULL, 0, result, NULL, 0, rpn, precision) && finite(*result))
      error = true;

   if (error && !kinematic_error_reported_)
      {
      sprintf(mesg, "%c Cannot evaluate expression %s", axisName_, expr);
      pC_->setCtrlError(mesg);
      kinematic_error_reported_ = true;
      return asynError;
      }

    return asynSuccess;
}

/* These are the functions for profile moves */
asynStatus GalilCSAxis::initializeProfile(size_t maxProfilePoints)
{
  if (profilePositions_)       free(profilePositions_);
  profilePositions_ =         (double *)calloc(maxProfilePoints, sizeof(double));
  return asynSuccess;
}

/** Set the high limit position of the motor.
  * \param[in] highLimit The new high limit position that should be set in the hardware. Units=steps.*/
asynStatus GalilCSAxis::setHighLimit(double highLimit)
{
  string mesg = "";		//Controller mesg
  //this gets called at init for every mR

  //Store high limit
  highLimit_ = highLimit;
  //Check if soft limits disabled
  if (highLimit_ == 0.0 && lowLimit_ == 0.0) {
     //Soft limits are disabled
     mesg = string(1, axisName_) + " soft limits disabled";
  }

  //Write controller mesg
  if (!mesg.empty())
     pC_->setCtrlError(mesg);

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Set the low limit position of the motor.
  * \param[in] lowLimit The new low limit position that should be set in the hardware. Units=steps.*/
asynStatus GalilCSAxis::setLowLimit(double lowLimit)
{
  string mesg = "";		//Controller mesg

  //Store low limit
  lowLimit_ = lowLimit;
  //Check if soft limits disabled
  if (highLimit_ == 0.0 && lowLimit_ == 0.0) {
     //Soft limits are disabled
     mesg = string(1, axisName_) + " soft limits disabled";
  }

  //Write controller mesg
  if (!mesg.empty())
     pC_->setCtrlError(mesg);

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus GalilCSAxis::poller(void)
{
   //static const char *functionName = "GalilAxis::poll";
   GalilAxis *pAxis;            //Galil real axis
   int slipstall, csslipstall;  //Encoder slip stall following error for each motor, and overall cs axis 
   double mres;                 //Motor resolution
   bool moving;                 //Moving status
   int homed;                   //Real motor homed status
   int csrev, csfwd;            //Determined cs axis limit status
   int cshomed;                 //Determined cs axis homed status
   int csmoving;                //Coordinate system axis moving status derived from real axis moving status
   bool csinmotion;             //Coordinate system axis in motion (ignores deferred moves, is real motion)
   int rdmov;                   //Any real motor doing backlash or retry
   int rmoving;                 //Real motor moving status
   int rhoming;                 //Real motor homing status
   int status;                  //Communication status with controller
   unsigned i;                  //Looping

   //Default communication status
   status = asynError;
   //Default moving status
   moving = 0;
   //Default slipstall status
   csslipstall = slipstall = 0;
   //cs axis limit status
   csrev = csfwd = 0;
   //Default moving status
   csmoving = 0;
   //Default inmotion status
   csinmotion = false;
   //Default homed status
   cshomed = homed = 0;
   //Default homing status
   cshoming_ = rhoming = 0;

   //Perform forward kinematic transform using real axis readback data, variable values, and
   //store results in GalilCSAxis, or asyn ParamList
   status = forwardTransform();
   if (status) goto skip;

   //Get real axis status, and work out what to propagate to CSAxis
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      if (!pAxis) continue;
      //Retrieve moving status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->motorStatusMoving_, &rmoving);
      //Retrieve dmov status.  Real motor may be doing backlash or retry
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilDmov_, &rdmov);
      //Or moving status from all real axis to derive cs moving status
      csmoving |= (deferredMove_ || (!rdmov && !pAxis->deferredMove_) || \
                  (rmoving && !pAxis->deferredMove_) || (pAxis->deferredMove_));
      //Or inmotion status from all real axis to derive cs inmotion status.  Ignore CSAxis deferredMove
      csinmotion |= pAxis->inmotion_;
      //Retrieve stall/following error status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->motorStatusSlip_, &slipstall);
      //Or slipstall from all real axis to derive cs slipstall status
      csslipstall |= slipstall;
      //Or homing status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHoming_, &rhoming);
      cshoming_ |= rhoming;
      //Set CSAxis limits
      if (pAxis->fwd_ && limitOrientation_[i] == consistent)
         csfwd |= 1;
      else if (pAxis->fwd_)
         csrev |= 1;
      if (pAxis->rev_ && limitOrientation_[i] == consistent)
         csrev |= 1;
      else if (pAxis->rev_)
         csfwd |= 1;
      //Retrieve homed status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->motorStatusHomed_, &homed);
      //Check cs axis homed status
      if (!i)
         cshomed = homed;
      else
         cshomed &= homed;
      }

   //Enforce CSAxis completion order
   csmoving |= enforceCSAxisCompletionOrder(csmoving);

   //Moving status
   moving = (csmoving) ? true : false;
   //Done status
   done_ = (moving) ? false : true;

   //Calculate CSAxis direction
   if (motor_position_ > last_motor_position_ + 1)
      direction_ = 1;
   else if (motor_position_ < last_motor_position_ - 1)
      direction_ = 0;

   //Monitor CSAxis move, stop if problem
   monitorCSAxisMove();

   //Clear CSAxis dynamics at move completion
   clearCSAxisDynamics();

   //Update CSAxis setpoint after jog completed
   if (moveVelocity_ && done_)
      {
      setPoint_ = motor_position_;
      //Reset moveVelocity flag
      moveVelocity_ = false;
      }

   //Save motor position and done status for next poll cycle
   last_motor_position_ = motor_position_;
   last_done_ = done_;

   //Show axis as moving until 1 cycle after axis ready
   //This is done so correct setpoint is given in motor record postProcess at startup
   if (!lastaxisReady_)
      {
      moving = true;
      done_ = false;
      csrev = false;
      csfwd = false;
      }

   //Set CSAXis initial setpoint
   if (!lastaxisReady_ && axisReady_)
      setPoint_ = motor_position_;

   //Set CSAxis setpoint after homing
   if (!cshoming_ && last_cshoming_)
      setPoint_ = motor_position_;

   lastaxisReady_ = axisReady_;
   last_cshoming_ = cshoming_;

skip:
   //Set status
   //Homing status flag
   //This flag does include JAH
   setIntegerParam(pC_->GalilHoming_, cshoming_);
   //Pass step count/aux encoder info to motorRecord
   setDoubleParam(pC_->motorPosition_, motor_position_);
   //Pass encoder value to motorRecord
   setDoubleParam(pC_->motorEncoderPosition_, encoder_position_);
   //Pass home, and limits status to motorRecord
   setIntegerParam(pC_->motorStatusAtHome_, 0);
   setIntegerParam(pC_->motorStatusHome_, 0);
   //Set cs axis limit status
   setIntegerParam(pC_->motorStatusLowLimit_, csrev);
   setIntegerParam(pC_->motorStatusHighLimit_, csfwd);
   //Set homed status
   setIntegerParam(pC_->motorStatusHomed_, cshomed);
   //Pass stall status to higher layers
   setIntegerParam(pC_->motorStatusSlip_, csslipstall);
   //Pass direction to motorRecord
   setIntegerParam(pC_->motorStatusDirection_, direction_);
   //Pass moving status to motorRecord
   setIntegerParam(pC_->motorStatusDone_, done_);
   setIntegerParam(pC_->motorStatusMoving_, moving);
   //Pass comms status to motorRecord
   setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
   //Pass CSAxis setpoint to upper layers
   pC_->getDoubleParam(axisNo_, pC_->motorResolution_, &mres);
   setDoubleParam(pC_->GalilCSMotorSetPoint_, setPoint_*mres);
   //Update motor status fields in upper layers using asynMotorAxis->callParamCallbacks
   callParamCallbacks();
   //Status delivered to MR, now send events to waiting threads
   sendAxisEvents(csinmotion);
   //Always return success. Dont need more error mesgs
   return asynSuccess;
}

