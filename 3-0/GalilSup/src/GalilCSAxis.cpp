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
#include <Galil.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <typeinfo>  //std::bad_typeid

using namespace std; //cout ostringstream vector string

#include <epicsString.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <errlog.h>
#include "sCalcPostfix.h"
#include "postfix.h"

#include <asynOctetSyncIO.h>

#include "GalilController.h"
#include <epicsExport.h>

// These are the GalilCSAxis methods

/** Creates a new GalilCSAxis object.
  */
GalilCSAxis::GalilCSAxis(class GalilController *pC, 	//The GalilController
	     char axisname,				//The coordinate system axis name I-P
	     char *csaxes, 				//List of coordinate system axis
	     char *forward,				//Forward kinematic transform used to calculate the coordinate system axis position from real axis positions
             char *fwdvars, 				//Forward kinematic variables List of Q-X
	     char *fwdsubs, 				//Forward kinematic substitutes List of A-P
             char *axes,                        	//List of real axis
	     char **reverse,				//Reverse transforms to calculate each real axis position in the coordinate system
	     char **revvars,				//Reverse kinematic variables List of Q-X
	     char **revsubs)				//Reverse kinematic substitutes List of A-P
  : asynMotorAxis(pC, (toupper(axisname) - AASCII)),
    pC_(pC)
{
  unsigned i;
  //store axis details
  //Store axis name
  axisName_ = (char)(toupper(axisname));

  //set defaults
  setDefaults();
  //Store list of real axis
  raxes_ = epicsStrDup(axes);
  //Store list of coordinate system axis
  csaxes_ = epicsStrDup(csaxes);
  //Store forward kinematic transform for this coordinate system axis
  forward_ = epicsStrDup(forward);
  //Store forward kinematic variables
  fwdvars_ = epicsStrDup(fwdvars);
  //Store forward kinematic substitutes
  fwdsubs_ = epicsStrDup(fwdsubs);
  //Store reverse transforms for the real axis
  reverse_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char));
  //Store reverse transforms variables
  revvars_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char));
  //Store reverse transforms substitutes
  revsubs_ = (char **)calloc(MAX_GALIL_AXES, sizeof(char));

  for (i = 0; i < strlen(raxes_); i++)
	{
	//Store reverse transforms for the real axis
	reverse_[i] = epicsStrDup(reverse[i]);
	//Store reverse transforms variables
	revvars_[i] = epicsStrDup(revvars[i]);
	//Store reverse transforms substitutes
	revsubs_[i] = epicsStrDup(revsubs[i]);
	}
}

/*--------------------------------------------------------------------------------*/
/* Store settings, set defaults for motor */
/*--------------------------------------------------------------------------------*/

asynStatus GalilCSAxis::setDefaults(void)
{
  //const char *functionName = "GalilCSAxis::setDefaults";

  //Set encoder stall flag to false
  setIntegerParam(pC_->GalilEStall_, 0);
  //Store axis in ParamList
  setIntegerParam(pC_->GalilAxis_, axisNo_);
  // We assume motor with encoder
  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  callParamCallbacks();

  //Give default readback values for positions, movement direction
  motor_position_ = 0;
  encoder_position_ = 0;
  direction_ = 1;
  //This coordinate system axis is not actually using coordinate system S or T right now
  coordsys_ = -1;

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
  unsigned i;
  double nmotor_positions[MAX_GALIL_AXES];	//New motor position targets for real axis
  GalilAxis *pAxis;				//Real GalilAxis
  int status = asynError;

  //Perform reverse transform and get new motor positions
  status = reverseTransform(position, nmotor_positions);

  //Select a free coordinate system
  if ((coordsys_ = selectFreeCoordinateSystem()) == -1)
	return asynError;

  //Do the coordinate system axis move, using deferredMoves facility
  if (!status)
	{
	//Set controller deferred move in paramList
	pC_->setIntegerParam(pC_->motorDeferMoves_, 1);
	//Now set controller deferred move flag
	pC_->setDeferredMoves(true);

	for (i = 0; i < strlen(raxes_); i++)
		{
		//Retrieve the GalilAxis real axis
		pAxis = pC_->getAxis(raxes_[i] - AASCII);
		if (!pAxis) continue;

		//Move the motors
		pAxis->move(nmotor_positions[i], relative, minVelocity, maxVelocity, acceleration);
		}

	//Clear controller deferred move in paramList
	pC_->setIntegerParam(pC_->motorDeferMoves_, 0);
	//Clear controller deferred move flag, and start motion
	pC_->setDeferredMoves(false);
        //Allow time for motion to begin
        epicsThreadSleep(.1);
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
  //Do the "jog"
  move(position, 0, minVelocity, fabs(maxVelocity), acceleration);
  //Allow time for motion to begin
  epicsThreadSleep(.02);

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}


/** Stop the motor.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus GalilCSAxis::stop(double acceleration)
{
  static const char *functionName = "GalilCSAxis::stop";

  //Stop the coordinate system S or T that this coordinate axis started
  if (coordsys_ == 0 || coordsys_ == 1) 
	{
        sprintf(pC_->cmd_, "ST %c\n", (coordsys_ == 0) ? 'S' : 'T');
        pC_->writeReadController(functionName);
	}

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Select a free coordinate system, or return -1
  */
int GalilCSAxis::selectFreeCoordinateSystem(void)
{
  static const char *functionName = "GalilCSAxis::selectFreeCoordinateSystem";
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
		status = pC_->writeReadController(functionName);
		//Proceed if coordsys change ok
		if (status)
			return -1;
		//Set change in coordsys in paramList
		pC_->setIntegerParam(0, pC_->GalilCoordSys_, coordsys);
		}
	}

  return coordsys;
}

/* Given axis list, retrieve readbacks and pack into margs (motor position) and eargs (encoder position)
 * \param[in] axes - Axis list
 * \param[in] margs - Motor position related arguments
 * \param[in] eargs - Encoder position related arguments
*/
asynStatus GalilCSAxis::packKinematicArgs(char *axes, double margs[], double eargs[])
{
   unsigned i;			//Looping
   double mpos, epos;		//Motor, and encoder readback data
   int status = asynSuccess;

   //Retrieve readbacks for all axis and pack into margs, eargs
   //equation provided by user
   for (i = 0; i < strlen(axes); i++)
	{
        //Get the readbacks for the axis
	status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorEncoderPosition_, &epos);
	status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorPosition_, &mpos);

	//Pack motor positions for calc
	if (!status)
		margs[axes[i] - AASCII] = mpos;
	//Pack encoder positions for calc
	if (!status)
		eargs[axes[i] - AASCII] = epos;
	}

   return (asynStatus)status;
}

/* Peform reverse kinematic transform using coordinate system axis readback data, and new position from user
 * for this coordinate system axis
 * And calculate real motor positions
 * \param[in] nposition - New motor position for this coordinate system axis
 * \param[out] motor_positions - Calculated motor positions for the real axis*/
int GalilCSAxis::reverseTransform(double nposition, double nmotor_positions[])
{
  double mpos;				//motor position for the related coordinate system saxis
  double margs[SCALCARGS];		//Coordinate system axis motor positions used in the transform
  double eargs[SCALCARGS];		//Coordinate system axis encoder positions used in the transform
  unsigned i, j;
  int status = asynSuccess;

  //Pack args for csaxes
  status |= packKinematicArgs(csaxes_, margs, eargs);

  //Substitute new motor position received for this csaxis, instead of using readback
  margs[axisName_ - AASCII] = nposition;

  //Pack args for real axes
  status |= packKinematicArgs(raxes_, margs, eargs);

  for (i = 0; i < strlen(raxes_); i++)
	{
	//Get variable values specified
	//and pack them in margs for the reverse transform calculation
	for (j = 0; j < strlen(revvars_[i]); j++)
		{
		//Get the variable values. Q-Z variables stored in addr 0-9
		status |= pC_->getDoubleParam(revvars_[i][j] - QASCII, pC_->GalilCoordSysVar_, &mpos);
		//Pack variable positions for motor calc
		if (!status)
			margs[revsubs_[i][j] - AASCII] = mpos;
		}

	if (!status)
		{
		//Perform reverse kinematic calc to derive the required real axis positions
		status |= doCalc(reverse_[i], margs, &nmotor_positions[i]);
		}
	}
	
  return status;
}

//Peform forward kinematic transform using readback data, variables and store results in GalilCSAxis as csaxis readback
int GalilCSAxis::forwardTransform(void)
{
   //const char *functionName="GalilCSAxis::getStatus";
  unsigned i;
  double margs[SCALCARGS];	//Motor positions used in the forward transform
  double eargs[SCALCARGS];	//Encoder positions used in the forward transform
  double mpos, epos;		//Real axis motor and encoder position in steps/counts
  int status = asynSuccess;	//Asyn paramList return code

  //Pack args for real axes
  status |= packKinematicArgs(raxes_, margs, eargs);

  //Pack args for csaxes
  status |= packKinematicArgs(csaxes_, margs, eargs);

  //Get variable values specified
  //and pack them in margs, eargs for the forward transform calculation
  for (i = 0; i < strlen(fwdvars_); i++)
	{
	//Get the variable values. Q-Z variables stored in addr 0-9
	status |= pC_->getDoubleParam(fwdvars_[i] - QASCII, pC_->GalilCoordSysVar_, &epos);
	status |= pC_->getDoubleParam(fwdvars_[i] - QASCII, pC_->GalilCoordSysVar_, &mpos);
	//Pack variable positions for motor calc
	if (!status)
		margs[fwdsubs_[i] - AASCII] = mpos;
	//Pack variable positions for encoder calc
	if (!status)
		eargs[fwdsubs_[i] - AASCII] = epos;
	}

  //Perform forward kinematic calc to get cs axis readback data
  if (!status)
	{
	//Motor position
	status |= doCalc(forward_, margs, &motor_position_);
	//Encoder position
	status |= doCalc(forward_, eargs, &encoder_position_);
	}

  return status;
}

//Perform kinematic calculations
asynStatus GalilCSAxis::doCalc(const char *expr, double args[], double *result) {
    /* Evaluate expression, return result */
    unsigned char rpn[512];	//Expression is converted to reverse polish notation 
    short err;
    
    *result = 0.0;
    *result /= *result;  /* Start as NaN */

    //We use sCalcPostfix and sCalcPerform because it can handle upto 16 args
    if (sCalcPostfix(expr, rpn, &err)) {
	printf("sCalcPostfix error in expression %s \n", expr);
	return asynError;
    } else 
	if (sCalcPerform(args, SCALCARGS, NULL, 0, result, NULL, 0, rpn) && finite(*result)) {
	    printf("calcPerform: error evaluating '%s'", expr);
	    return asynError;
    }

    return asynSuccess;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * callParamCallbacks is now called in GalilPoller
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus GalilCSAxis::poll(bool *moving)
{
   //static const char *functionName = "GalilAxis::poll";
   int done;					//Done status
   bool motor_move;				//motor move move status
   int slipstall, csslipstall;			//Encoder slip stall following error for each motor, and overall cs axis 
   int rev, fwd;				//Real motor rev and fwd limit status
   int last_csrev, last_csfwd;			//cs axis limit status prior to this poll cycle
   int csrev, csfwd;				//Determined cs axis limit status
   int rmoving, csmoving;			//Real axis moving status, coordinate system axis moving status
   int status;			 		//Communication status with controller
   unsigned i;					//Looping

   //Default communication status
   status = asynError;
   //Default moving status
   *moving = 0;
   done = 1;
   motor_move = false;
   //Default slipstall status
   csslipstall = slipstall = 0;
   //cs axis limit status
   csrev = csfwd = 0;
   //Default moving status
   csmoving = rmoving = 0;
   //Retrieve cs axis previous poll cycle limit status
   pC_->getIntegerParam(axisNo_, pC_->motorStatusLowLimit_, &last_csrev);
   pC_->getIntegerParam(axisNo_, pC_->motorStatusHighLimit_, &last_csfwd);
   
   //Peform forward kinematic transform using real axis readback data, variable values, and
   //store results in GalilCSAxis, or asyn ParamList
   status = forwardTransform();
   if (status) goto skip;

   //Determine cs axis movement direction
   if (motor_position_ > last_motor_position_)
	direction_ = 1;
   else if (motor_position_ < last_motor_position_)
	direction_ = 0;

   //Save cs axis motor position for next poll cycle
   last_motor_position_ = motor_position_;

   //Get real axis limits, and work out what to propagate to the cs axis
   //use cs axis direction_, previous cs axis limit status, and axis limit status to work out cs axis limit status
   //Also propagate ANY encoder slip/stall status, and move status to cs axis
   for (i = 0; i < strlen(raxes_); i++)
	{
	//Retrieve moving status
	status = pC_->getIntegerParam(raxes_[i] - AASCII, pC_->motorStatusMoving_, &rmoving);
	//Or moving status from all real axis to derive cs moving status
        csmoving |= rmoving;
	//Retrieve limit status
	status |= pC_->getIntegerParam(raxes_[i] - AASCII, pC_->motorStatusLowLimit_, &rev);
	status |= pC_->getIntegerParam(raxes_[i] - AASCII, pC_->motorStatusHighLimit_, &fwd);
	//Retrieve stall/following error status
	status |= pC_->getIntegerParam(raxes_[i] - AASCII, pC_->motorStatusSlip_, &slipstall);
	//Or slipstall from all real axis to derive cs slipstall status
	csslipstall |= slipstall;
	if (!status)
		{
		//Check cs axis limit status
		//Check cs axis reverse limit
		if ((!direction_ && rev) || (last_csrev && direction_ && rev) ||
		    (!direction_ && fwd) || (last_csrev && direction_ && fwd))
			{
			csrev |= 1;
			}

		//Check cs axis forward limit
		if ((direction_ && fwd) || (last_csfwd && !direction_ && fwd) ||
		    (direction_ && rev) || (last_csfwd && !direction_ && rev))
			{
			csfwd |= 1;
			}
		}
	}

   //Moving status
   *moving = (bool)csmoving;
   done = (*moving) ? false : true;

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

