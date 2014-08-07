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

#ifndef GalilAxis_H
#define GalilAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define KPMAX		      1023.875
#define KDMAX		      4095.875

class GalilAxis : public asynMotorAxis
{
public:

  GalilAxis(class GalilController *pC, 
	    char *axisname,			/*axisname A-H */
	    int limit_as_home,			/*0=no, 1=yes. Using a limit switch as home*/
	    char *enables_string,		/*digital input(s) to use to send home, or interlock motor if -ve*/
	    int switch_type);			/*digital input to use to send away from home*/

  //These are the methods that are new to this class

  //Store settings, and implement defaults
  asynStatus setDefaults(int limit_as_home, char *enables_string, int switch_type);

  //Store motor digital inhibits
  void store_motors_enable(void);

  //Initialize code generator
  void initialize_codegen(char axis_thread_code[],
			  char axis_limit_code[],
			  char axis_dighome_code[]);

  //Generate code for limits interrupt
  void gen_limitcode(char c,
		     char axis_thread_code[],
		     char axis_limit_code[]);

  //Generate code for digital input interrupt
  void gen_digitalcode(char c,
		       int digitalhome,
		       int digitalaway,
		       char axis_dighome_code[]);

   //Generate axis home routine
   void gen_homecode(char c, char axis_thread_code[]);
   
   //Is the motor in an enabled/go state with current digital IO status
   bool motor_enabled(void);
   //Check SSI reading against disconnect value
   void set_ssi_connectflag(void);
   //Set SSI setting on controller
   asynStatus set_ssi(void);
   //Get SSI setting from controller
   asynStatus get_ssi(int function, epicsInt32 *value);
   //Set limdc parameter on controller
   asynStatus setLimitDecel(double velocity);
   //Extract axis data from GalilController data record
   asynStatus getStatus(void);

  /* These are the methods we override from the base class */
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setEncoderRatio(double ratio);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setClosedLoop(bool closedLoop);

private:
  GalilController *pC_;      		/**< Pointer to the asynMotorController to which this axis belongs.
                                	*   Abbreviated because it is used very frequently */
  char axisName_;			//The axis letter A-H
  bool limit_as_home_;	     		//use limit switches for homing
  int switch_type_;			//switch type for motor enable/disable function
  char *enables_string_;		//Motor enable/disable string specified by user
  int invert_ssi_;			//Invert ssi encoder.  Reverse -ve, and +ve direction of ssi
  double motor_position_;		//aux encoder or step count register
  double encoder_position_;		//main encoder register
  double last_motor_position_;		//aux encoder or step count register stored from previous poll.  Used to detect movement.
  double last_encoder_position_;	//main encoder register stored from previous poll.  Used to detect movement.
  int direction_;			//Movement direction
  bool inmotion_;			//Axis in motion status from controller
  bool protectStop_;			//Used to flag that protected stop has occurred already
  bool fwd_;				//Forward limit status
  bool rev_;				//Reverse limit status
  bool home_;				//Home status
  double highLimit_;			//High soft limit
  double lowLimit_;			//Low soft limit
  double encmratio_;			//Encoder/motor ratio
  bool encmratioset_;			//Flag to indicate if the ratio has been set
  epicsTimeStamp pestall_nowt_;		//Used to track length of time encoder has been stalled for
  epicsTimeStamp pestall_begint_;	//Time when possible encoder stall first detected
  bool pestall_detected_;		//Possible encoder stall detected flag
  int deferredCoordsys_;		//Coordinate system 0 (S) or 1 (T)
  double deferredAcceleration_;		//Coordinate system acceleration
  double deferredVelocity_;		//Coordinate system velocity
  double deferredPosition_;		//Deferred move position
  bool deferredMove_;			//Has a deferred move been set
  
friend class GalilController;
};

#endif   // GalilAxis_H

