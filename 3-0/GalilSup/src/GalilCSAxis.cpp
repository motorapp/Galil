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
  encoder_position_ = 0;
  direction_ = 1;
  //This coordinate system axis is not actually using coordinate system S or T right now
  coordsys_ = -1;
  //The coordinate system is not stopping on a limit switch right now
  stop_onlimit_ = false;
  //A move has not been started by this cs axis
  move_started_ = false;
  //Axis not ready until necessary motor record fields have been pushed into driver
  //So we use "use encoder if present" UEIP field to set axisReady_ to true
  axisReady_ = false;

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
  GalilAxis *pAxis;				//Real motor
  int status = asynError;

  //Clear stop on limit before move
  stop_onlimit_ = false;

  //A move has been started by this cs axis
  move_started_ = true;

  //Perform reverse transform and get new motor positions
  status = reverseTransform(position, nmotor_positions);

  //Select a free coordinate system
  if ((coordsys_ = selectFreeCoordinateSystem()) == -1)
	return asynError;

  //Do the coordinate system axis move, using deferredMoves facility in GalilController
  if (!status)
	{
	//Now set controller deferred move flag
	pC_->setDeferredMoves(true);
        epicsThreadSleep(.001);
	
	for (i = 0; i < strlen(revaxes_); i++)
		{
		//Retrieve the axis
                pAxis = pC_->getAxis(revaxes_[i] - AASCII);
		if (!pAxis) continue;
		//Move the motor
		pAxis->move(nmotor_positions[i], relative, minVelocity, maxVelocity, acceleration);
		}

	//Clear controller deferred move flag, and start motion
	pC_->setDeferredMoves(false);
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

/* Given axis list, retrieve readbacks and pack into margs (motor position) and eargs (encoder position)
 * \param[in] axes - Axis list
 * \param[in] margs - Motor position related arguments
 * \param[in] eargs - Encoder position related arguments
*/
asynStatus GalilCSAxis::packKinematicArgs(char *axes, double margs[], double eargs[])
{
  unsigned i;			//Looping
  double mpos, epos;		//Motor, and encoder readback data
  double mres, eres;		//Motor record mres, eres
  int status = asynSuccess;

  //Retrieve readbacks for all axis and pack into margs, eargs
  //equation provided by user
  for (i = 0; i < strlen(axes); i++)
     {
     //Get the readbacks for the axis
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorEncoderPosition_, &epos);
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorPosition_, &mpos);
     //Retrieve needed motor record fields
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->motorResolution_, &mres);
     status |= pC_->getDoubleParam(axes[i] - AASCII, pC_->GalilEncoderResolution_, &eres);

     //Pack motor positions for calc in dial coordinates
     if (!status)
        margs[axes[i] - AASCII] = mpos * mres;
        
     //Pack encoder positions for calc in dial coordinates
     if (!status)
        eargs[axes[i] - AASCII] = epos * eres;
     }

  return (asynStatus)status;
}

/* Peform reverse kinematic transform using coordinate system axis readback data, and new position from user
 * for this coordinate system axis
 * And calculate real motor positions
 * \param[in] nposition - New motor position for this coordinate system axis
 * \param[out] nmotor_positions - Calculated motor positions for the real axis*/
int GalilCSAxis::reverseTransform(double nposition, double nmotor_positions[])
{
  double mpos;			//motor position for the related coordinate system saxis
  double mres;			//motor record mres
  double margs[SCALCARGS];	//Coordinate system axis motor positions used in the transform
  double eargs[SCALCARGS];	//Coordinate system axis encoder positions used in the transform
  unsigned i, j;
  int status = asynSuccess;

  //Pack args for forward axes
  status |= packKinematicArgs(fwdaxes_, margs, eargs);

  //Pack args for reverse axes
  status |= packKinematicArgs(revaxes_, margs, eargs);

  //Retrieve needed motor record parameters
  status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);
  //Substitute new motor position received for this csaxis, instead of using readback
  //Also convert position into dial coordinates
  margs[axisName_ - AASCII] = nposition * mres;

  for (i = 0; i < strlen(revaxes_); i++)
	{
	//Get variable values specified
	//and pack them in margs for the reverse transform calculation
	for (j = 0; j < strlen(revvars_[i]); j++)
		{
		//Get the variable values. Q-Z variables stored in addr 0-9
		status |= pC_->getDoubleParam(revvars_[i][j] - QASCII, pC_->GalilCSMotorVariable_, &mpos);
		//Pack variable positions for motor calc
		if (!status)
			margs[revsubs_[i][j] - AASCII] = mpos;
		}
	//Retrieve needed motor record parameters
	status |= pC_->getDoubleParam(revaxes_[i] - AASCII, pC_->motorResolution_, &mres);

	if (!status)
		{
		//Perform reverse kinematic calc to derive the required real axis positions
		status |= doCalc(reverse_[i], margs, &nmotor_positions[i]);
		//Convert dial position value back into steps
		nmotor_positions[i] = nmotor_positions[i]/mres;
		}
	}
	
  return status;
}

//Perform forward kinematic transform using readback data, variables and store results in GalilCSAxis as csaxis readback
int GalilCSAxis::forwardTransform(void)
{
   //const char *functionName="GalilCSAxis::getStatus";
  unsigned i;
  double margs[SCALCARGS];	//Motor positions used in the forward transform
  double eargs[SCALCARGS];	//Encoder positions used in the forward transform
  double mpos, epos;		//Motor and encoder position in steps/counts
  double mres, eres;		//Motor record mres, and eres
  int status = asynSuccess;	//Asyn paramList return code

  //Pack args for reverse axes
  status |= packKinematicArgs(revaxes_, margs, eargs);

  //Pack args for forward axes
  status |= packKinematicArgs(fwdaxes_, margs, eargs);

  //Get variable values specified
  //and pack them in margs, eargs for the forward transform calculation
  for (i = 0; i < strlen(fwdvars_); i++)
	{
	//Get the variable values. Q-Z variables stored in addr 0-9
	status |= pC_->getDoubleParam(fwdvars_[i] - QASCII, pC_->GalilCSMotorVariable_, &epos);
	status |= pC_->getDoubleParam(fwdvars_[i] - QASCII, pC_->GalilCSMotorVariable_, &mpos);
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
	//Retrieve needed motor record fields
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->motorResolution_, &mres);
	status |= pC_->getDoubleParam(axisName_ - AASCII, pC_->GalilEncoderResolution_, &eres);
	//Calculate motor position readback data in dial coordinates
	status |= doCalc(forward_, margs, &motor_position_);
	//Convert from dial to steps for interaction with motor record
	motor_position_ = motor_position_/mres;
	//Calculate encoder position readback data in dial coordinates
	status |= doCalc(forward_, eargs, &encoder_position_);
	//Convert from dial to steps
	encoder_position_ = encoder_position_/eres;
	}

  return status;
}

//Perform kinematic calculations
//Evaluate expression, return result
asynStatus GalilCSAxis::doCalc(const char *expr, double args[], double *result) {
   
   unsigned char rpn[512];		//Expression is converted to reverse polish notation 
   short err;
   bool error = false;			//Error status
   char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg
    
   *result = 0.0;

   //For empty expressions
   if (!strcmp(expr, ""))
      return asynSuccess;

   //We use sCalcPostfix and sCalcPerform because it can handle upto 16 args
   if (sCalcPostfix(expr, rpn, &err))
      error = true;
   else if (sCalcPerform(args, SCALCARGS, NULL, 0, result, NULL, 0, rpn) && finite(*result))
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
   GalilAxis *pAxis;		//Galil real axis
   int done;			//Done status
   int slipstall, csslipstall;	//Encoder slip stall following error for each motor, and overall cs axis 
   int rev, fwd;		//Real motor rev and fwd limit status
   int csrev, csfwd;		//Determined cs axis limit status
   int rmoving, csmoving;	//Real axis moving status, coordinate system axis moving status derived from real axis moving status
   int status;			//Communication status with controller
   bool reportlimits;		//Do we report limits for this csaxis under current circumstances
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
   status = forwardTransform();
   if (status) goto skip;

   //Determine cs axis movement direction
   if (motor_position_ > last_motor_position_)
	direction_ = 1;
   else if (motor_position_ < last_motor_position_)
	direction_ = 0;

   //Get axis limits, and work out what to propagate to the cs axis
   //Also propagate ANY encoder slip/stall status, and move status to cs axis
   for (i = 0; i < strlen(revaxes_); i++)
	{
	//Retrieve the axis
	pAxis = pC_->getAxis(revaxes_[i] - AASCII);
	if (!pAxis) continue;
	//Check if real motor stopping on limit only if this cs axis started a move
	if ((pAxis->stop_code_ == 2 && move_started_) || (pAxis->stop_code_ == 3 && move_started_))
		stop_onlimit_ = true;
	//Retrieve moving status
	status = pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusMoving_, &rmoving);
	//Or moving status from all real axis to derive cs moving status
	csmoving |= rmoving;
	//Retrieve limit status
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusLowLimit_, &rev);
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusHighLimit_, &fwd);
	//Retrieve stall/following error status
	status |= pC_->getIntegerParam(revaxes_[i] - AASCII, pC_->motorStatusSlip_, &slipstall);
	//Or slipstall from all real axis to derive cs slipstall status
	csslipstall |= slipstall;
	if (!status)
		{
		//Report limits only when stop caused by limit and this cs axis started the move
		reportlimits = false;
		if (stop_onlimit_)
			reportlimits = true;
		//Check cs axis limit status
		//Check cs axis reverse limit
		if (!direction_ && reportlimits && (rev || fwd))
			csrev |= 1;
		//Check cs axis forward limit
		if (direction_ && reportlimits && (fwd || rev))
			csfwd |= 1;
		}
	}

   //Moving status
   *moving = (csmoving) ? true : false;
   //Done status
   done = (*moving) ? false : true;

   //Reset move started flag
   if (done)
      move_started_ = false;

   //Save motor position and done status for next poll cycle
   last_motor_position_ = motor_position_;
   last_done_ = done;

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

