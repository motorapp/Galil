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

  //Set encoder stall flag to false
  setIntegerParam(pC_->GalilEStall_, 0);
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
  //The coordinate system is not stopping on a limit switch right now
  stop_onlimit_ = false;
  //Sync start only mode requires stop to be issued for all CSAxis motor upon stop_onlimit_
  stop_issued_ = false;
  //A move has not been started by this cs axis
  move_started_ = false;
  //Axis not ready until necessary motor record fields have been pushed into driver
  //So we use "use encoder if present" UEIP field to set axisReady_ to true
  lastaxisReady_ = axisReady_ = false;

  return asynSuccess;
}

/** Check requested motor velocities
  * \param[in] npos - After kinematics these are the requested motor positions Units=Steps
  * \param[in] nvel - After kinematics these are the requested  motor velocities Units=Steps/s */
asynStatus GalilCSAxis::checkMotorVelocities(double npos[], double nvel[])
{
  unsigned i;				//Looping
  double incmove[MAX_GALIL_AXES];	//Real motor relative move distances
  char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg
  double mres, eres;			//Motor record mres, eres
  int ueip;				//Motor record ueip
  double mpos, epos;			//Motor position, encoder position for real motor
  double vectorVelocity = 0;		//Computed vector velocity
  double vectorDistance = 0;		//Computed vector distance
  double vel;				//Temp variable used to calculate real motor velocity
  double vmax;				//Motor record vmax

  if (deferredMode_)
     {
     //Sync start and stop uses linear mode
     //Use vector mathematics to check requested motor velocities
     //Loop thru all real motors, calculate vector distance and velocity
     for (i = 0; i < strlen(revaxes_); i++)
        {
        //Get the readbacks for the axis
        pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorEncoderPosition_, &epos);
        pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorPosition_, &mpos);
        //Retrieve needed motor record fields
        pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilUseEncoder_, &ueip);
        //Calculate incremental move distance in steps
        incmove[i] = (ueip) ? npos[i] - epos : npos[i] - mpos;
        //Sum vector distance, velocity for non zero move increments
        if (fabs(incmove[i]) != 0.0)
           {
           //Calculate vector distance
           vectorDistance += pow(incmove[i], 2);
           //Calculate vector velocity
           vectorVelocity += pow(nvel[i], 2);
           }
        }

     //Calculate vector distance
     vectorDistance = sqrt(vectorDistance);
     //Calculate vector velocity
     vectorVelocity = sqrt(vectorVelocity);

     //Calculate actual motor speeds as controller does in linear mode
     for (i = 0; i < strlen(revaxes_); i++)
        {
        //Retrieve needed motor record fields
        pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilMotorVmax_, &vmax);
        pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilUseEncoder_, &ueip);
        pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
        pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilEncoderResolution_, &eres);
        //Calculate this motors actual velocity in egu
        vel = (incmove[i]/vectorDistance) * vectorVelocity;
        vel = (ueip) ? vel * eres : vel * mres;
        if (fabs(vel) > fabs(vmax))
           {
           sprintf(mesg, "Move failed, axis %c velocity %lf > VMAX %lf\n", revaxes_[i], fabs(vel), fabs(vmax));
           pC_->setCtrlError(mesg);
           return asynError;
           }
        }
    }
  else
    {
    //Sync start only mode
    //Loop thru all real motors and check velocity
    for (i = 0; i < strlen(revaxes_); i++)
       {
       //Retrieve needed motor record fields
       pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilMotorVmax_, &vmax);
       pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->GalilUseEncoder_, &ueip);
       pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
       pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->GalilEncoderResolution_, &eres);
       //Calculate requested velocity in egu
       vel = (ueip) ? nvel[i] * eres : nvel[i] * mres;
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
  GalilCSAxis *pCSAxis;			//Pointer to GalilCSAxis instance
  unsigned i, j = 0;			//Looping, indexing
  double npos[MAX_GALIL_AXES];		//Real axis position targets
  double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
  double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
  CSTargets targets;			//Addtional CSAxis targets
  GalilAxis *pAxis;			//Pointer to GalilAxis instance
  int deferredMode;			//Deferred move mode
  string axes = "";			//Construct a real axis list
  int status = asynError;

  //Clear stop on limit before move
  stop_onlimit_ = false;
  //Clear stop issued flag before move
  stop_issued_ = false;

  //Retrieve deferred moves mode
  pC_->getIntegerParam(pC_->GalilDeferredMode_, &deferredMode);
  deferredMode_ = deferredMode;

  //Are moves to be deferred ?
  if (pC_->movesDeferred_ != 0)
	{
	//Store parameters for deferred move in GalilCSAxis
	deferredPosition_ = position;
	pC_->getIntegerParam(0, pC_->GalilCoordSys_, &deferredCoordsys_);
	deferredVelocity_ = maxVelocity;
	deferredAcceleration_ = acceleration;
	deferredRelative_ = relative;

	//Scan for other CSAxis that have a new position setpoint too
	for (i = 0; i < strlen(fwdaxes_); i++)
		{
		//Retrieve the related CSAxis instance
		pCSAxis = pC_->getCSAxis(fwdaxes_[i] - AASCII);
		if (pCSAxis)
			{
			//See if retrieved CSAxis instance has a new position setpoint
			if (pCSAxis->deferredCoordsys_ == deferredCoordsys_ && pCSAxis->deferredMove_)
				{
				//Another related CSAxis has a new target also, store it
				//Requested position
				targets.ncspos[j] = pCSAxis->deferredPosition_;
				//Requested velocity
				targets.ncsvel[j] = pCSAxis->deferredVelocity_;
				//Requested acceleration
				targets.ncsaccel[j] = pCSAxis->deferredAcceleration_;
		               //Store the axis in the list
				targets.csaxes[j++] = pCSAxis->axisName_;
				}
			}
		}

	//Perform reverse transform and get new axis (real) motor positions, velocities
	status = reverseTransform(position, maxVelocity, acceleration, &targets, npos, nvel, naccel);

	//Check requested motor velocities
	status |= checkMotorVelocities(npos, nvel);

	//Write the motor setpoints, but dont move
	if (!status)
		{
		for (i = 0; i < strlen(revaxes_); i++)
			{
			//Retrieve the axis
			pAxis = pC_->getAxis(revaxes_[i] - AASCII);
			if (!pAxis) continue;
			//Write motor set point, but dont start motion
			pAxis->move(npos[i], relative, minVelocity, nvel[i], naccel[i]);
			}
		deferredMove_ = true;
		move_started_ = true;
		}
	
	return asynSuccess;
	}
  else
	{
	//Moves are not deferred
	//Perform reverse transform and get new motor positions
	status = reverseTransform(position, maxVelocity, acceleration, NULL, npos, nvel, naccel);

	//Check requested motor velocities
	status |= checkMotorVelocities(npos, nvel);

	//Select a free coordinate system
	if (deferredMode && !status)
	   if (selectFreeCoordinateSystem() == -1)
		status = asynError;

	//Do the coordinate system axis move, using deferredMoves facility in GalilController
	if (!status)
		{
		//Set controller deferred move flag
		pC_->setDeferredMoves(true);
		epicsThreadSleep(.001);
		}

	//Write the motor setpoints, but dont move
	if (!status)
		{
		for (i = 0; i < strlen(revaxes_); i++)
			{
			//Retrieve the axis
			pAxis = pC_->getAxis(revaxes_[i] - AASCII);
			if (!pAxis) continue;
			//Write motor set point, but dont start motion
			pAxis->move(npos[i], relative, minVelocity, nvel[i], naccel[i]);
			}
		//Clear controller deferred move flag, and start motion
		pC_->setDeferredMoves(false);
		move_started_ = true;
		}
	}

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

  //Choose large move distance to simulate jog function
  position = (maxVelocity > 0) ? 838860 : -838860;

  //Convert position to relative
  position = motor_position_ + position;

  //Do the "jog"
  move(position, 0, minVelocity, fabs(maxVelocity), acceleration);
  
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
  for (i = 0; i < (unsigned)strlen(revaxes_); i++)
     {
     //Retrieve the axis
     pAxis = pC_->getAxis(revaxes_[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //cancel any home operations that may be underway
     sprintf(pC_->cmd_, "home%c=0", pAxis->axisName_);
     pC_->sync_writeReadController();
     pAxis->homing_ = false;
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

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Move the motors to the home position.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus GalilCSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
   static const char *functionName = "GalilCSAxis::home";
   unsigned i;		//Looping
   GalilAxis *pAxis;	//GalilAxis
   double npos[MAX_GALIL_AXES];		//Real axis position targets
   double nvel[MAX_GALIL_AXES];		//Real axis velocity targets
   double naccel[MAX_GALIL_AXES];	//Real axis acceleration targets
   int dir;				//Reverse axis home direction
   int hometypeallowed;			//Home type allowed
   int status;				//Driver status
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg

   //Perform reverse transform to get real motor accelerations and velocities
   //We dont care about position here (arbitary 100)
   status = reverseTransform(100, maxVelocity, acceleration, NULL, npos, nvel, naccel);

   if (!status)
      {
      //Loop thru real motor list, and send them home
      for (i = 0; i < (unsigned)strlen(revaxes_); i++)
         {
         //Retrieve axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;

         //Ensure motor is good to go
         if (pAxis->beginCheck(functionName, nvel[i])) return asynError;

         //Retrieve home type allowed for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHomeAllowed_, &hometypeallowed);
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
         //Home axis in direction we worked out above
         if (hometypeallowed)
            {
            //set acceleration, velocity is ignored here
            pAxis->setAccelVelocity(naccel[i], maxVelocity);
            //Set home velocity, direction.  We dont use return hvel here
            pAxis->setupHome(nvel[i], dir);
            }
         else
            {//This reverse axis does not allow homing at all
            sprintf(mesg, "%c axis extra settings do not allow homing", pAxis->axisName_);
            pC_->setCtrlError(mesg);
            return asynSuccess;
            }
         }
   
      //Start motors simultanously
      pC_->beginGroupMotion(revaxes_);
      
      //Loop thru real motor list, and set home flags
      for (i = 0; i < (unsigned)strlen(revaxes_); i++)
         {
         //Retrieve axis
         pAxis = pC_->getAxis(revaxes_[i] - AASCII);
         //Process or skip
         if (!pAxis) continue;
         //Retrieve home type allowed for this axis
         pC_->getIntegerParam(pAxis->axisNo_, pC_->GalilHomeAllowed_, &hometypeallowed);
         //If revaxes allows homing, set flags now home jog has started
         if (hometypeallowed)
            {
            pAxis->homing_ = true;  //Start was successful
            pAxis->cancelHomeSent_ = false;  //Homing has not been cancelled yet
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
  for (i = 0; i < strlen(revaxes_); i++)
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

/* Given axis list, retrieve readbacks and pack into mrargs (motor readback args)
 * \param[in] axes - Axis list
 * \param[out] mrargs - Motor readback arguments Units=EGU
*/
asynStatus GalilCSAxis::packReadbackArgs(char *axes, double mrargs[])
{
  unsigned i;			//Looping
  double mpos, epos;		//Motor, and encoder readback data
  double mres, eres;		//Motor record mres, eres
  int ueip;			//Use encoder if present setting
  int status = asynSuccess;

  //Retrieve readbacks for all axis and pack into mrargs
  //equation provided by user
  for (i = 0; i < strlen(axes); i++)
     {
     //Get the readbacks for the axis
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorEncoderPosition_, &epos);
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorPosition_, &mpos);
     //Retrieve needed motor record fields
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorResolution_, &mres);
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->GalilEncoderResolution_, &eres);
     status |= pC_->getIntegerParam(axes[i] - AASCII, pC_->GalilUseEncoder_, &ueip);

     //Pack motor readbacks for calc in egu dial coordinates
     if (!status)
        mrargs[axes[i] - AASCII] = (ueip) ? (epos * eres) : (mpos * mres);
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
 * \param[out] naccel - Calculated motor accelerations for the real axis Units-Steps/s/s*/
asynStatus GalilCSAxis::reverseTransform(double pos, double vel, double accel, CSTargets *targets, double npos[], double nvel[], double naccel[])
{
  double mres, eres;		//Motor record mres, eres
  int ueip;			//Motor record use encoder if present setting
  double ctargs[SCALCARGS];	//Coordinate transform arguments
  double vtargs[SCALCARGS];	//Velocity transform arguments
  double atargs[SCALCARGS];	//Acceleration transform arguments
  double value;			//Kinematic arg value
  unsigned i, j;
  int status = asynSuccess;

  //Default velocity, acceleration transform args to zero
  for (i = 0; i < SCALCARGS; i++)
     {
     vtargs[i] = 0;
     atargs[i] = 0;
     ctargs[i] = 0;
     }

  //Pack position readback args for forward axes (ie. related CSAxis)
  status |= packReadbackArgs(fwdaxes_, ctargs);

  //Add other CSAxis position, velocity, acceleration setpoints to args if supplied
  if (targets != NULL)
     for (i = 0; i < strlen(targets->csaxes); i++)
        {
        //Retrieve needed motor record parameters for related CSAxis
        status |= pC_->getDoubleParam(targets->csaxes[i] - AASCII, pC_->motorResolution_, &mres);
        //Add position for csaxis in egu
        ctargs[targets->csaxes[i] - AASCII] = (targets->ncspos[i] * mres);
        //Add velocity for csaxis in egu
        vtargs[targets->csaxes[i] - AASCII] = (targets->ncsvel[i] * mres);
        //Add acceleration for csaxis in egu
        atargs[targets->csaxes[i] - AASCII] = (targets->ncsaccel[i] * mres);
        }

  //Retrieve needed motor record parameters for this CSAxis
  status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);
  status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->GalilEncoderResolution_, &eres);
  status |= pC_->getIntegerParam(axisName_ - AASCII, pC_->GalilUseEncoder_, &ueip);

  //Substitute new motor position received for this CSAxis, instead of using readback
  //Convert new position from steps into egu dial coordinates
  ctargs[axisName_ - AASCII] = (pos * mres);
  //Add CSAXis velocity in dial egu
  vtargs[axisName_ - AASCII] = (vel * mres);
  //Add CSAXis acceleration in dial egu
  atargs[axisName_ - AASCII] = (accel * mres);

  for (i = 0; i < strlen(revaxes_); i++)
	{
	//Get kinematic variable values specified
	//and pack them in mrargs for the reverse transform calculation
	for (j = 0; j < strlen(revvars_[i]); j++)
		{
		//Get the variable values. Q-Z variables stored in addr 0-9
		status |= pC_->getDoubleParam(revvars_[i][j] - QASCII, pC_->GalilCSMotorVariable_, &value);
		//Pack variable positions for motor calc
		if (!status)
			ctargs[revsubs_[i][j] - AASCII] = value;
		}
	//Retrieve needed motor record parameters
	status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);

	if (!status)
		{
		//Perform reverse coordinate transform to derive the required real axis positions 
		//given new csaxis position
		status |= doCalc(reverse_[i], ctargs, &npos[i]);
		//Convert dial position value back into steps for move
		npos[i] = npos[i]/mres;
		//Perform reverse velocity transform to derive the required real axis velocity 
		//given csaxis move velocity
		status |= doCalc(reverse_[i], vtargs, &nvel[i]);
		//Convert velocity value back into steps for move
		nvel[i] = fabs(nvel[i]/mres);
		//Perform reverse acceleration transform to derive the required real axis acceleration 
		//given csaxis move acceleration
		status |= doCalc(reverse_[i], atargs, &naccel[i]);
		//Convert acceleration value back into steps for move
		naccel[i] = naccel[i]/mres;
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
  double mres;			//Motor record encoder resolution, motor resolution
  int status;			//Return status
  int useCSAxis;		//useAxis flag for CSAxis
  int nPoints;			//Number of points in profile

  //Retrieve required attributes from ParamList
  pC_->getIntegerParam(0, pC_->profileNumPoints_, &nPoints);
  pC_->getIntegerParam(axisNo_, pC_->profileUseAxis_, &useCSAxis);
  //Process or skip this CSAxis
  if (!useCSAxis)
     return asynSuccess;

  //Retrieve the revaxes (real motors) in this CSAxis
  for (i = 0; i < strlen(revaxes_); i++)
     {
     pAxis[i] = pC_->getAxis(revaxes_[i] - AASCII);
     if (!pAxis[i]) //Return error if any GalilAxis not instantiated
        return asynError;
     else //Transform will proceed, and later restore is required for this GalilAxis
        pAxis[i]->restoreProfile_ = true;
     }

   //Transform this CSAxis, and create Axis profile data
   for (i = 0; i < (unsigned)nPoints; i++)
      {
      //Perform reverse transform and get new axis (real) motor positions
      //We dont care about velocity (arbitary 100) and acceleration (arbitary 100) here
      status = reverseTransform(profilePositions_[i], 100, 100, NULL, npos, nvel, naccel);
      //Copy new axis profile data point into revaxes GalilAxis instances
      for (j = 0; j < strlen(revaxes_); j++)
           {
           //Backup GalilAxis profile data
           pAxis[j]->profileBackupPositions_[i] = pAxis[j]->profilePositions_[i];
           //Copy in new GalilAxis profile data from transform
           pAxis[j]->profilePositions_[i] = npos[j];
           }
      }//Transform complete

   //Upload new points to database readback waveforms
   for (i = 0; i < strlen(revaxes_); i++)
      {
      //Retrieve needed motor record fields
      status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);
      //Copy profile into calculatedPositions array
      for (j = 0; j < (unsigned)nPoints; j++)
         {
         pAxis[i]->calculatedPositions_[j] = pAxis[i]->profilePositions_[j] * mres;
         }
      //Update calculatedPositions record for this revaxis (real motor)
      pC_->doCallbacksFloat64Array(pAxis[i]->calculatedPositions_, nPoints, pC_->GalilProfileCalculated_, revaxes_[i] - AASCII);
      }

  //All ok
  return asynSuccess;
}

//Perform forward kinematic transform using readback data, variables and store results in GalilCSAxis as csaxis readback
asynStatus GalilCSAxis::forwardTransform(void)
{
  unsigned i;
  double mrargs[SCALCARGS];	//Motor readback args in egu used in the forward transform
  double value;			//Kinematic arg value
  double mres, eres;		//Motor record mres, and eres
  int ueip;			//Motor record use encoder if present
  int status = asynSuccess;	//Asyn paramList return code

  //Pack position readback args for reverse axes (ie. real motor readbacks)
  status |= packReadbackArgs(revaxes_, mrargs);

  //Get variable values specified
  //and pack them in mrargs for the forward transform calculation
  for (i = 0; i < strlen(fwdvars_); i++)
	{
	//Get the kinematic variable values. Q-Z variables stored in addr 0-9
	status |= pC_->getDoubleParam(fwdvars_[i] - QASCII, pC_->GalilCSMotorVariable_, &value);
	//Pack variable positions for transform calc
	if (!status)
		mrargs[fwdsubs_[i] - AASCII] = value;
	}

  //Perform forward kinematic calc to get csaxis readback data
  if (!status)
	{
	//Retrieve needed motor record fields
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->GalilEncoderResolution_, &eres);
	status |= pC_->getIntegerParam(axisName_ - AASCII, pC_->GalilUseEncoder_, &ueip);
	//Calculate motor position readback data in dial coordinates
	if (!status)
		{
		if (ueip)
			{
			status |= doCalc(forward_, mrargs, &encoder_position_);
			//Convert from dial to steps for interaction with motor record
			encoder_position_ = encoder_position_/eres;
			}
		else
			{
			status |= doCalc(forward_, mrargs, &motor_position_);
			//Convert from dial to steps for interaction with motor record
			motor_position_ = motor_position_/mres;
			}
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
   int precision = 6;			//Hard code precision for now
    
   *result = 0.0;

   //For empty expressions
   if (!strcmp(expr, ""))
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
  if (profileReadbacks_)    free(profileReadbacks_);
  profileReadbacks_ =         (double *)calloc(maxProfilePoints, sizeof(double));
  if (profileFollowingErrors_) free(profileFollowingErrors_);
  profileFollowingErrors_ =   (double *)calloc(maxProfilePoints, sizeof(double));
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus GalilCSAxis::poll(bool *moving)
{
   //static const char *functionName = "GalilAxis::poll";
   GalilAxis *pAxis;		//Galil real axis
   int done;			//Done status
   int slipstall, csslipstall;	//Encoder slip stall following error for each motor, and overall cs axis 
   int rev, fwd;		//Real motor rev and fwd limit status
   int csrev, csfwd;		//Determined cs axis limit status
   int rmoving, csmoving;	//Real axis moving status, coordinate system axis moving status derived from real axis moving status
   int status;			//Communication status with controller
   unsigned i;			//Looping

   //Default communication status
   status = asynError;
   //Default moving status
   *moving = 0;
   done = 1;
   //Default slipstall status
   csslipstall = slipstall = 0;
   //cs axis limit status
   csrev = csfwd = 0;
   //Default moving status
   csmoving = rmoving = 0;

   //Perform forward kinematic transform using real axis readback data, variable values, and
   //store results in GalilCSAxis, or asyn ParamList
   if (axisReady_)
       status = forwardTransform();
   if (status) goto skip;

   //Determine moving, and stall status
   for (i = 0; i < strlen(revaxes_); i++)
	{
	//Retrieve the axis
	pAxis = pC_->getAxis(revaxes_[i] - AASCII);
	if (!pAxis) continue;
	//Retrieve moving status
	status = pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusMoving_, &rmoving);
	//Or moving status from all real axis to derive cs moving status
	csmoving |= ((rmoving && !pAxis->deferredMove_) || deferredMove_);
	//Retrieve stall/following error status
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusSlip_, &slipstall);
	//Or slipstall from all real axis to derive cs slipstall status
	csslipstall |= slipstall;
	}

   //Moving status
   *moving = (csmoving) ? true : false;
   //Done status
   done = (*moving) ? false : true;
   //Reset move started flag
   if (done)
      move_started_ = false;
   
   //Determine cs axis movement direction only if moving
   if (motor_position_ > last_motor_position_ && *moving)
	direction_ = 1;
   else if (motor_position_ < last_motor_position_ && *moving)
	direction_ = 0;

   //Get axis limits, and work out what to propagate to the cs axis
   for (i = 0; i < strlen(revaxes_); i++)
	{
	//Retrieve the axis
	pAxis = pC_->getAxis(revaxes_[i] - AASCII);
	if (!pAxis) continue;
	//Check if real motor stopping on limit only if this cs axis started a move
	if ((pAxis->stop_code_ == MOTOR_STOP_FWD && move_started_) || (pAxis->stop_code_ == MOTOR_STOP_REV && move_started_))
		stop_onlimit_ = true;
	//Don't report limits if a real axis in csaxis is moving independently
	stop_onlimit_ = (*moving && !move_started_) ? false : stop_onlimit_;
	//Retrieve limit status
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusLowLimit_, &rev);
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusHighLimit_, &fwd);
	if (!status)
		{
		//Check cs axis reverse limit
		if (!direction_ && stop_onlimit_ && (rev || fwd))
			csrev |= 1;
		//Check cs axis forward limit
		if (direction_ && stop_onlimit_ && (fwd || rev))
			csfwd |= 1;
		}
	}

   //Sync start only mode
   //If limit struck whilst moving, we must issue stop for all motors in CSAxis
   if ((csrev || csfwd) && *moving)
      {
      if (!deferredMode_ && !stop_issued_)
         {
         stop(1);
         stop_issued_ = true;
         }
      }

   //Save motor position and done status for next poll cycle
   last_motor_position_ = motor_position_;
   last_done_ = done;

   //Show axis as moving until 1 cycle after axis ready
   //This is done so correct setpoint is given in motor record at startup
   if (!lastaxisReady_)
      {
      *moving = true;
      done = false;
      }
   lastaxisReady_ = axisReady_;

skip:
    //Set status
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
    //Pass stall status to higher layers
    setIntegerParam(pC_->motorStatusSlip_, csslipstall);
    //Pass direction to motorRecord
    setIntegerParam(pC_->motorStatusDirection_, direction_);
    //Pass moving status to motorRecord
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, *moving);
    //Pass comms status to motorRecord
    setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
    //Update motor status fields in upper layers using asynMotorAxis->callParamCallbacks
    callParamCallbacks();
    //Always return success. Dont need more error mesgs
    return asynSuccess;
}

