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
// mark.clift@synchrotron.org.au
// 800 Blackburn Road, Clayton, Victoria 3168, Australia.
//

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <typeinfo>  //std::bad_typeid
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

// These are the GalilCSAxis methods

/** Creates a new GalilCSAxis object.
  */
GalilCSAxis::GalilCSAxis(class GalilController *pC, 	//The GalilController
	     char axisname)
  : asynMotorAxis(pC, (toupper(axisname) - AASCII)),
    pC_(pC)
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
}

//GalilAxis destructor
GalilCSAxis::~GalilCSAxis()
{
   unsigned i;

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
}

/*--------------------------------------------------------------------------------*/
/* Store settings, set defaults for motor */
/*--------------------------------------------------------------------------------*/

asynStatus GalilCSAxis::setDefaults(void)
{
  //const char *functionName = "GalilCSAxis::setDefaults";
  unsigned i;

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
  //Pass default step count/aux encoder value to motorRecord
  setDoubleParam(pC_->motorPosition_, motor_position_);
  //Pass default encoder value to motorRecord
  setDoubleParam(pC_->motorEncoderPosition_, encoder_position_);
  //Pass default direction value to motorRecord
  setIntegerParam(pC_->motorStatusDirection_, direction_);
  //Motor not homing now
  //This flag does include JAH
  setIntegerParam(pC_->GalilHoming_, 0);
  //Used to stop all motors in this CSAxis
  //Dont stop the CSAxis now
  stop_csaxis_ = false;
  //CSAxis has not started a move
  move_started_ = false;
  //CSAxis move is not a jog move
  moveVelocity_ = false;
  //Axis not ready until necessary motor record fields have been pushed into driver
  //So we use "use encoder if present" UEIP field to set axisReady_ to true
  lastaxisReady_ = axisReady_ = false;
  //This axis is not performing a deferred move
  deferredMove_ = false;

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
  double aCorrection;		//Correction when vector shorter than acceleration distance
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
           if (fabs(vel) > fabs(vmax))
              {
              sprintf(mesg, "Move failed, axis %c velocity %lf > VMAX %lf\n", revaxes_[i], fabs(vel), fabs(vmax));
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
       if (fabs(vel) > fabs(vmax))
           {
           sprintf(mesg, "Move failed, axis %c velocity %2.21lf > VMAX %2.21lf\n", revaxes_[i], fabs(vel), fabs(vmax));
           pC_->setCtrlError(mesg);
           return asynError;
           }
       }
    }

  //All ok
  strcpy(mesg, "");
  pC_->setCtrlError(mesg);

  return asynSuccess;
}

//Check status of reverse axis enable interlock
asynStatus GalilCSAxis::beginCheck(void)
{
   GalilAxis *pAxis;
   unsigned i;
   //Check reverse axis enable interlock
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      if (!pAxis) continue;
      //Check interlock, supply arbitary velocity
      //Dont reset controller message
      if (pAxis->beginCheck("move", 100, false))
         return asynError;
      }
   //Return value
   return asynSuccess;
}

//Check related axis motor record status
asynStatus GalilCSAxis::validateMRSettings(void)
{
   GalilCSAxis *pCSAxis;	//Related forward axis
   GalilAxis *pAxis;		//Reverse axis
   unsigned i;			//Looping

   //Check reverse axis motor record status
   for (i = 0; revaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pAxis = pC_->getAxis(revaxes_[i] - AASCII);
      if (!pAxis) continue;
      //Check motor record settings
      if (pAxis->checkMRSettings(moveVelocity_, axisName_))
         return asynError;
      }

   //Check forward axis motor record status
   for (i = 0; fwdaxes_[i] != '\0'; i++)
      {
      //Retrieve the axis
      pCSAxis = pC_->getCSAxis(fwdaxes_[i] - AASCII);
      if (!pCSAxis) continue;
      //Check motor record settings
      if (pCSAxis->checkMRSettings(moveVelocity_, axisName_))
         return asynError;
      }
   //Return value
   return asynSuccess;
}

//Check axis motor record settings
//Return error if record is not ready for new moves
asynStatus GalilCSAxis::checkMRSettings(bool moveVelocity, char callaxis)
{
   char mesg[MAX_GALIL_STRING_SIZE];
   int spmg;
   int set;
   //Retrieve needed params
   pC_->getIntegerParam(axisNo_, pC_->GalilStopPauseMoveGo_, &spmg);
   pC_->getIntegerParam(axisNo_, pC_->GalilMotorSet_, &set);
   //Check motor record status
   if (spmg != 3 && spmg != 2)
      sprintf(mesg, "%c move failed, %c spmg is not set to \"go\" or \"move\"", callaxis, axisName_);
   if (set && !moveVelocity)
      sprintf(mesg, "%c move failed, %c set field is not set to \"use\"", callaxis, axisName_);
   //Update controller message
   if ((spmg != 3 && spmg != 2) || (set && !moveVelocity))
      {
      pC_->setCtrlError(mesg);
      //Stop if any error
      return asynError;
      }
   //All good to go
   return asynSuccess;
}

/** Check CSAxis limit in requested move direction
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).*/
asynStatus GalilCSAxis::checkLimits(double position, int relative)
{
   double readback;	//CSAxis readback Units = steps
   int rev, fwd;	//CSAxis limit status
   int ueip;		//CSAxis use encoder if present
   int status;		//Return status

   //Retrieve needed motor record parameters
   status = pC_->getIntegerParam(axisNo_, pC_->GalilUseEncoder_, &ueip);
   status |= pC_->getIntegerParam(axisNo_, pC_->motorStatusLowLimit_, &rev);
   status |= pC_->getIntegerParam(axisNo_, pC_->motorStatusHighLimit_, &fwd);
   if (!status)
      {
      //Readback in steps
      readback = (ueip) ? encoder_position_ : motor_position_;
      //Check forward
      if ((relative && position < 0 && rev) || (!relative && position < readback && rev))
         status = asynError;
      //Check reverse
      if ((relative && position > 0 && fwd) || (!relative && position > readback && fwd))
         status = asynError;
      }

   //Return status
   return (asynStatus)status;
}

//Monitor CSAxis move, stop if problem
//Called by poller
asynStatus GalilCSAxis::monitorCSAxisMove(bool moving)
{
   GalilAxis *pAxis;	//Reverse axis
   int sc;		//Reverse axis stop code
   bool done = !moving; //CSAxis done
   unsigned i;		//Looping

   //Check reverse axis move status
   //Stop CSAxis if problem
   if (moving && move_started_ && !deferredMove_)
      {
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
         //Retrieve reverse axis instance
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Skip or continue
         if (!pAxis) continue;
         //Set axis stop code
         sc = pAxis->stop_code_;
         //Set flag if an axis stops
         if ((sc == MOTOR_STOP_FWD && pAxis->done_) || (sc == MOTOR_STOP_REV && pAxis->done_) ||
            (sc == MOTOR_STOP_STOP && pAxis->done_))
            stop_csaxis_ = true;
         }
      }

   //Clear stop axis flag when move complete
   //Dont wait for backlash, retries completion
   if (stop_csaxis_ && done)
      stop_csaxis_ = false;

   //Stop CSAxis if requested
   if (moving && stop_csaxis_ && move_started_)
      {
      //Stop the real motors in the CSAxis
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
         //Retrieve the axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;
         //Tell GalilAxis::poller to stop the axis
         pAxis->stop_axis_ = true;
         }
      }

   return asynSuccess;
}

//Clear CSAxis move dynamics at move completion
//Called by poller
asynStatus GalilCSAxis::clearCSAxisDynamics(bool moving)
{
   GalilAxis *pAxis;	//Reverse axis
   bool done = !moving; //CSAxis done
   unsigned i;		//Looping

   if (move_started_ && done)
      {
      //CSAxis move completed
      move_started_ = false;
      //After move clear axis useCSA flag
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
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
   int moving = 0;	//CSAxis moving status

   //Set deferred move flag
   deferredMove_ = true;
   //Unlock so synchronous poller is free
   pC_->unlock();
   //Wait till CSAxis moving status becomes true
   while (!moving)
      {
      pC_->getIntegerParam(axisNo_, pC_->motorStatusMoving_, &moving);
      if (!moving)
         epicsThreadSleep(0.001);
      }
   pC_->lock();
   
   //Now CSAxis moving status true
   //Set flag that move started
   move_started_ = true;

   //Set moveVelocity if instructed now that CSAxis moving status true
   if (moveVelocity)
      moveVelocity_ = true;

   return asynSuccess;
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
  //static const char *functionName = "GalilCSAxis::move";
  int i;				//Looping, indexing
  double npos[MAX_GALIL_AXES];		//Real axis position targets
  double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
  double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
  int moveStatus[MAX_GALIL_AXES];	//Real axis move thru motor record status
  int lastaxis = -1;			//Last axis in move when moves not deferred
  GalilAxis *pAxis;			//Pointer to GalilAxis instance
  bool setDeferred = false;
  int status = asynError;

  //Store CSAxis setpoint for use in kinematics
  if (!moveVelocity_)
     setPoint_ = position;
  //Clear controller message
  pC_->setCtrlError("");

  //Retrieve deferred moves mode
  pC_->getIntegerParam(pC_->GalilDeferredMode_, &deferredMode_);
  pC_->getIntegerParam(0, pC_->GalilCoordSys_, &deferredCoordsys_);
  deferredRelative_ = relative;

  //Perform reverse transform and get new motor positions
  status = reverseTransform(position, maxVelocity, acceleration, NULL, npos, nvel, naccel);
  //Check requested motor velocities
  status |= checkMotorVelocities(npos, nvel, naccel);
  //Check motor record settings
  status |= validateMRSettings();
  //check limit in requested move direction
  status |= checkLimits(position, relative);

  if (!pC_->movesDeferred_ && !status)
     {
     //Moves are not deferred
     //Check motor enable interlocks
     status |= beginCheck();

     //Select a free coordinate system
     if (deferredMode_ && !status)
        if (selectFreeCoordinateSystem() == -1)
           status |= asynError;

     if (!status)
        {
        //Set controller deferred move flag
        pC_->setDeferredMoves(true);
        setDeferred = true;
        epicsThreadSleep(.001);
        }
     }

  if (!status)
     {
     for (i = 0; revaxes_[i] != '\0'; i++)
        {
        //Retrieve the axis
        pAxis = pC_->getAxis(revaxes_[i] - AASCII);
        if (!pAxis) continue;
        //Default startDeferredMoves before writing new setpoint
        pAxis->startDeferredMoves_ = false;
        //Write motor set point, but dont start motion
        if (moveVelocity_)//Jog moves are accomplished through internal calls to GalilAxis move
           pAxis->move(npos[i], relative, minVelocity, nvel[i], naccel[i]);
        else
           {
           //Moves are sent to real axis motor records
           //Backlash, and retries are supported
           moveStatus[i] = pAxis->moveThruMotorRecord(npos[i], nvel[i], naccel[i]);
           //Track last axis with successful move
           if (!moveStatus[i])
              lastaxis = i;
           }
        }

     //For moves, not for jog
     //Setup method to start deferred moves
     if (!moveVelocity_ && lastaxis != -1)
        {
        //Retrieve the last axis with a successful move
        pAxis = pC_->getAxis(revaxes_[lastaxis] - AASCII);
        if (pAxis)
           {
           //This axis has successful move setup by moveThruMotorRecord
           //Flag this axis should start deferredMoves when GalilAxis::move is called
           if (setDeferred)
              pAxis->startDeferredMoves_ = true;
           //Set CSAxis move flags
           setupCSAxisMove(false);
           }
        }

     //For jog
     //Also for move when requested position < rdbd from readback
     //Must start deferred moves here
     if ((setDeferred && moveVelocity_) || (setDeferred && !deferredMove_))
        pC_->setDeferredMoves(false);
     }

  //Clear deferred move flag for failed jog
  //This can occur when move fails initial checks (eg. checkMotorVelocities)
  if (moveVelocity_ && status)
     deferredMove_ = false;

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

  //Retrieve deferred moves mode
  pC_->getIntegerParam(pC_->GalilDeferredMode_, &deferredMode);
  //Set max
  //In sync start stop mode this is the maximum incremental distance
  //In sync start only mode this is the maximum absolute position
  max = (deferredMode) ? MAX_GALIL_LINEAR_INCREMENT : MAX_GALIL_ABSOLUTE_MOVE/1.1;
  //Set initial attempt
  increment = max;
  //Reduce move size until within controller limitations
  while (!done)
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
     reverseTransform(position, maxVelocity, acceleration, NULL, npos, nvel, naccel);
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
     }

  if (!pC_->movesDeferred_)
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

  //Cancel home operations
  for (i = 0; revaxes_[i] != '\0'; i++)
     {
     //Retrieve the axis
     pAxis = pC_->getAxis(revaxes_[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //cancel any home operations that may be underway
     sprintf(pC_->cmd_, "home%c=0", pAxis->axisName_);
     pC_->sync_writeReadController();
     //Set homing flag false
     //This flag does not include JAH
     pAxis->homing_ = false;
     //This flag does include JAH
     pC_->setIntegerParam(pAxis->axisNo_, pC_->GalilHoming_, 0);
     //cancel any home switch jog off operations that may be underway
     sprintf(pC_->cmd_, "hjog%c=0", pAxis->axisName_);
     pC_->sync_writeReadController();
     }

  //Issue stop
  if (strcmp(revaxes_, "") != 0)
     {
     //revaxes_ cannot be empty, else all threads on controller get killed
     //Stop the real motors that this CSAxis started
     sprintf(pC_->cmd_, "ST %s", revaxes_);
     pC_->sync_writeReadController();
     }

  //Tell poller to keep issuing stop
  //This is to stop retries, backlash correction
  stop_csaxis_ = true;

  //Clear defer move flag for this axis
  if (deferredMove_)
     deferredMove_ = false;

  //Always return success. Dont need more error mesgs
  return asynSuccess;
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
   unsigned i;						//Looping
   unsigned j = 0, k = 0;			//Build axes lists
   GalilAxis *pAxis;				//GalilAxis
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

   //Default list of axes we move home
   strcpy(maxes, "");
   //Default list of axes we prepare to move home
   strcpy(paxes, "");

   //Perform reverse transform to get real motor accelerations and velocities
   //We dont care about position here (arbitary 100)
   status = reverseTransform(100, maxVelocity, acceleration, NULL, npos, nvel, naccel);

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
               //Ensure motor is good to go
               if (!i)
                  status = pAxis->beginCheck(functionName, nvel[i]);
               else
                  status = pAxis->beginCheck(functionName, nvel[i], false);
               if (status) return asynError; //Return if any problem
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
               epicsEventWaitWithTimeout(pAxis->stoppedTimeResetEventId_, pC_->updatePeriod_/1000.0);
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

   return asynSuccess;
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
   for (i = 0; i < strlen(equation); i++)
      {
      equation[i] = toupper(equation[i]);
      //Create list of axis found in equation
      if ((equation[i] >= 'A' && equation[i] <= 'P' && !isalpha(equation[i+1]) && !i) ||
         (i && equation[i] >= 'A' && equation[i] <= 'P' && !isalpha(equation[i+1]) && !isalpha(equation[i-1])))
         {
         //Ensure no CS motors in forward equations, and no real motors in reverse equations
         if ((equation[i] >= first && equation[i] <= last && !isalpha(equation[i+1]) && !i) ||
             (i && equation[i] >= first && equation[i] <= last && !isalpha(equation[i+1]) && !isalpha(equation[i-1])))
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
  unsigned i, j;						//Looping
  bool forward = (axis >= 'I' && axis <='P') ? true : false;	//Transform direction forward or reverse
  string equation_s = equation;					//For string substitution
  char subst_transform[MAX_GALIL_STRING_SIZE];			//The substitute transform		
  char first = (forward) ? 'I' : 'A';				//Axis we substitute for complete transform
  char last = (forward) ? 'P' : 'H';				//Axis we substitute for complete transform
  char mesg[MAX_GALIL_STRING_SIZE];				//Controller error mesg

  //Scan through transform equation
  for (j = 0; j < MAX_GALIL_CSAXES + 1; j++)
     for (i = 0; i < strlen(equation); i++)
        {
        equation[i] = toupper(equation[i]);
        if ((equation[i] >= first && equation[i] <= last && !isalpha(equation[i+1]) && !i) ||
            (i && equation[i] >= first && equation[i] <= last && !isalpha(equation[i+1]) && !isalpha(equation[i-1])))
           {
           //Found a motor we want to substitute with a complete transform
           //Retrieve the substitute transform
           if (forward)
              pC_->getStringParam(equation[i] - AASCII, pC_->GalilCSMotorForward_ , MAX_GALIL_STRING_SIZE, subst_transform);
           else
              pC_->getStringParam(axisNo_, pC_->GalilCSMotorReverseA_ + equation[i] - AASCII, MAX_GALIL_STRING_SIZE, subst_transform);

           //Substitute motor letter with complete transform
           equation_s.replace(i,i+1, subst_transform);
           strcpy(equation, equation_s.c_str());
           //If we keep finding things to substitute by this time there are either
           //Too many CS motors in forward equations or too many real motors in reverse equations 
           if ((j == MAX_GALIL_CSAXES) && axisReady_)
              {
              sprintf(mesg, "%c transform has too many substitutes", axis);
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

   //Only CS motors appear in reverse equations
   //Only Real motors appear in forward equations
   //Apply substitution to ensure this is true or fail
   status = substituteTransforms(axis, equation);
   //Create axes list from transform equation
   status |= obtainAxisList(axis, equation, axes);
   //Replace variables in range Q-X with variables in range A-P for sCalcPerform
   status |= substituteVariables(axis, equation, axes, vars, subs);
  
   return asynStatus(status);
}

//Parse the kinematic equations and load into GalilCSAxis instance
asynStatus GalilCSAxis::parseTransforms(void)
{
  unsigned i;
  int status;
  string fwdaxes_s = "";

  //Flag kinematic error not yet reported
  kinematic_error_reported_ = false;

  //Retrieve forward kinematic equation for this cs axis (eg. I=(A+B)/2)
  status = pC_->getStringParam(axisNo_, pC_->GalilCSMotorForward_ , MAX_GALIL_STRING_SIZE, forward_);
  //Parse forward transform into GalilCSAxis instance
  if (strcmp(forward_, ""))
     status |= parseTransform(axisName_, forward_, revaxes_, fwdvars_, fwdsubs_);
 
  //Retrieve reverse kinematic equations for axes found in forward transform retrieved above
  for (i = 0; revaxes_[i] != '\0'; i++)
     {
     //Retrieve reverse transform for axis specified in revaxes_
     status |= pC_->getStringParam(axisNo_, pC_->GalilCSMotorReverseA_ + revaxes_[i] - AASCII, MAX_GALIL_STRING_SIZE, reverse_[i]);
     //Parse reverse transform into GalilCSAxis instance
     if (strcmp(reverse_[i], ""))
        status |= parseTransform(revaxes_[i], reverse_[i], fwdaxes_, revvars_[i], revsubs_[i]);
     //Concat axis list found in last reverse transform
     fwdaxes_s += fwdaxes_;
     //Sort the axes list
     sort(fwdaxes_s.begin(), fwdaxes_s.end());
     //Remove any duplicates
     fwdaxes_s.erase(std::unique(fwdaxes_s.begin(), fwdaxes_s.end()), fwdaxes_s.end());
     //Copy into fwdaxes_
     strcpy(fwdaxes_, fwdaxes_s.c_str());
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
   int status = asynSuccess;    //Return status

   if (kinematicsAltered_)
      {
      for (i = 0; revaxes_[i] != '\0'; i++)
         {
         //Zero limit argument array
         for (j = 0; j < SCALCARGS; j++)
            largs[j] = 0;
         //Retrieve the axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         if (!pAxis) continue;
         //Insert 1 for this axis, all others are 0
         largs[revaxes_[i] - AASCII] = 1;
         //Perform reverse transform to calculate real axis limit orientation
         status |= doCalc(forward_, largs, &orientation);
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
 * \param[in] targets - Related CSAxis that have new setpoints too
 * \param[out] npos - Calculated motor positions for the real axis Units=Steps
 * \param[out] nvel - Calculated motor velocities for the real axis Units=steps/s
 * \param[out] naccel - Calculated motor accelerations for the real axis Units-Steps/s/s
 * \param[out] useCSSetpoints - Use CSAxis setpoints in transform if true, else use readbacks*/
asynStatus GalilCSAxis::reverseTransform(double pos, double vel, double accel, CSTargets *targets, double npos[], double nvel[], double naccel[], bool useCSSetpoints)
{
  double mres;			//Motor record mres
  double off;			//Motor record offset
  int dir, dirm;		//Motor record dir, and direction multiplier
  double ctargs[SCALCARGS];	//Coordinate transform arguments
  double vtargs[SCALCARGS];	//Velocity transform arguments
  double atargs[SCALCARGS];	//Acceleration transform arguments
  double lowest_accel = 99999999;
  long lowest_accelhw;
  double accel_ratio;
  unsigned i;
  int status = asynSuccess;

  //Default velocity, acceleration, coordinate transform args to zero
  for (i = 0; i < SCALCARGS; i++)
     {
     vtargs[i] = 0;
     atargs[i] = 0;
     ctargs[i] = 0;
     }

  //Pack forward axes setpoints (ie. related CSAxis)
  if (useCSSetpoints)
     status |= packSetPointArgs(ctargs);
  else//Use forward axes readbacks instead
     status |= packReadbackArgs(fwdaxes_, ctargs);

  //Add other CSAxis position, velocity, acceleration setpoints to args if supplied
  if (targets != NULL)
     for (i = 0; targets->csaxes[i] != '\0'; i++)
        {
        //Retrieve needed motor record parameters for related CSAxis
        status |= pC_->getDoubleParam(targets->csaxes[i] - AASCII, pC_->motorResolution_, &mres);
        //Add position for csaxis in user egu
        ctargs[targets->csaxes[i] - AASCII] = (targets->ncspos[i] * mres);
        //Add velocity for csaxis in user egu
        vtargs[targets->csaxes[i] - AASCII] = (targets->ncsvel[i] * mres);
        //Add acceleration for csaxis in user egu
        atargs[targets->csaxes[i] - AASCII] = (targets->ncsaccel[i] * mres);
        }

  //Retrieve needed motor record parameters for this CSAxis
  status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);

  //Substitute new motor position received for this CSAxis
  //Convert new position from steps into user coordinates
  //OFF, and DIR always 0 for csaxis
  ctargs[axisName_ - AASCII] = (pos * mres);
  //Add CSAXis velocity in dial egu
  vtargs[axisName_ - AASCII] = (vel * mres);
  //Add CSAXis acceleration in dial egu
  atargs[axisName_ - AASCII] = (accel * mres);

  for (i = 0; revaxes_[i] != '\0'; i++)
	{
	//Retrieve needed motor record parameters
	status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
	status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilUserOffset_, &off);
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilDirection_, &dir);
	//Calculate direction multiplier
	dirm = (dir == 0) ? 1 : -1;
	if (!status)
		{
		//Perform reverse coordinate transform to derive the required real axis positions 
		//given new csaxis position
		status |= doCalc(reverse_[i], ctargs, &npos[i]);
		//Convert position from user coordinates to steps for move
		npos[i] = (npos[i] - off)/(mres * dirm);
		//Perform reverse velocity transform to derive the required real axis velocity 
		//given csaxis move velocity
		status |= doCalc(reverse_[i], vtargs, &nvel[i]);
		//Convert velocity from egu to steps for move
		nvel[i] = fabs(nvel[i]/mres);
		//Perform reverse acceleration transform to derive the required real axis acceleration 
		//given csaxis move acceleration
		status |= doCalc(reverse_[i], atargs, &naccel[i]);
		//Convert acceleration from egu to steps for move
		naccel[i] = fabs(naccel[i]/mres);
		//Find lowest acceleration in the group
		if (naccel[i] < lowest_accel)
			lowest_accel = naccel[i];
		}
	}

  //Refactor acceleration given limited acceleration resolution on controller
  if (!status)
	{
	//Find closest hardware setting for lowest acceleration found
	lowest_accelhw = (long)lrint(lowest_accel/1024.0) * 1024;
	//Now refactor real motor acceleration
	for (i = 0; revaxes_[i] != '\0'; i++)
		{
		accel_ratio = naccel[i]/lowest_accel;
		naccel[i] = lowest_accelhw * accel_ratio;
		}
	}
	
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
      status = reverseTransform(profilePositions_[i], 100, 100, NULL, npos, nvel, naccel, false);
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
  if (!status)
	{
	//Retrieve needed motor record fields
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->GalilEncoderResolution_, &eres);
	//Do the forward transform
	if (!status)
		{
		//Perform forward coordinate transform
		status |= doCalc(forward_, mrargs, &position);
		//Coordinate transform results
		//CSAxis OFF, and DIR always 0
		//CSAxis dial, and user coordinates are the same
		//Convert from user coordinates to steps for interaction with motor record
		encoder_position_ = position/eres;
		//Convert from user coordinates to steps for interaction with motor record
		motor_position_ = position/mres;
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

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus GalilCSAxis::poller(void)
{
   //static const char *functionName = "GalilAxis::poll";
   GalilAxis *pAxis;		//Galil real axis
   int done;			//Done status
   int slipstall, csslipstall;	//Encoder slip stall following error for each motor, and overall cs axis 
   bool moving;			//Moving status
   int homed;			//Real motor homed status
   int csrev, csfwd;		//Determined cs axis limit status
   int cshomed;			//Determined cs axis homed status
   int csmoving;		//Coordinate system axis moving status derived from real axis moving status
   int rdmov;			//Any real motor doing backlash or retry
   int rmoving;			//Real motor moving status
   int cshoming;		//CSAxis homing status
   int rhoming;			//Real motor homing status
   int status;			//Communication status with controller
   unsigned i;			//Looping

   //Default communication status
   status = asynError;
   //Default moving status
   moving = 0;
   done = 1;
   //Default slipstall status
   csslipstall = slipstall = 0;
   //cs axis limit status
   csrev = csfwd = 0;
   //Default moving status
   csmoving = 0;
   //Default homed status
   cshomed = homed = 0;
   //Default homing status
   cshoming = rhoming = 0;

   //Perform forward kinematic transform using real axis readback data, variable values, and
   //store results in GalilCSAxis, or asyn ParamList
   if (axisReady_)
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
      csmoving |= (deferredMove_ || !rdmov  || rmoving);
      //Retrieve stall/following error status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->motorStatusSlip_, &slipstall);
      //Or slipstall from all real axis to derive cs slipstall status
      csslipstall |= slipstall;
      //Or homing status
      status |= pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHoming_, &rhoming);
      cshoming |= rhoming;
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

   //Moving status
   moving = (csmoving) ? true : false;
   //Done status
   done = (moving) ? false : true;

   //Calculate CSAxis direction
   if (motor_position_ > last_motor_position_ + 1)
      direction_ = 1;
   else if (motor_position_ < last_motor_position_ - 1)
      direction_ = 0;

   //Monitor CSAxis move, stop if problem
   monitorCSAxisMove(moving);

   //Clear CSAxis dynamics at move completion
   clearCSAxisDynamics(moving);

   //Update CSAxis setpoint after jog completed
   if (moveVelocity_ && done)
      {
      setPoint_ = motor_position_;
      //Reset moveVelocity flag
      moveVelocity_ = false;
      }

   //Save motor position and done status for next poll cycle
   last_motor_position_ = motor_position_;
   last_done_ = done;

   //Show axis as moving until 1 cycle after axis ready
   //This is done so correct setpoint is given in motor record postProcess at startup
   if (!lastaxisReady_)
      {
      moving = true;
      done = false;
      csrev = false;
      csfwd = false;
      }

   //Set CSAXis initial setpoint
   if (!lastaxisReady_ && axisReady_)
      setPoint_ = motor_position_;

   lastaxisReady_ = axisReady_;

skip:
   //Set status
   //Homing status flag
   //This flag does include JAH
   setIntegerParam(pC_->GalilHoming_, cshoming);

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
   setIntegerParam(pC_->motorStatusDone_, done);
   setIntegerParam(pC_->motorStatusMoving_, moving);
   //Pass comms status to motorRecord
   setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
   //Update motor status fields in upper layers using asynMotorAxis->callParamCallbacks
   callParamCallbacks();
   //Always return success. Dont need more error mesgs
   return asynSuccess;
}

