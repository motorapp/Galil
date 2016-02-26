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
#include "epicsMessageQueue.h"

#define KPMAX			1023.875
#define KDMAX			4095.875
#define HOMING_TIMEOUT		3.5

//Home type allowed
#define HOME_NONE 0
#define HOME_REV 1
#define HOME_FWD 2
#define HOME_BOTH 3

//pollServices request numbers
static const int MOTOR_STOP = 0;
static const int MOTOR_POST = 1;
static const int MOTOR_OFF = 2;
static const int MOTOR_HOMED = 3;
static const int MOTOR_CANCEL_HOME = 4;
static const int MOTOR_BRAKE_ON = 5;

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
  //Set acceleration and velocity
  asynStatus setAccelVelocity(double acceleration, double velocity);
  //Extract axis data from GalilController data record
  asynStatus getStatus(void);
  //Set poller status variables bassed on GalilController data record info
  void setStatus(bool *moving);
  //Verify encoder operation whilst moving for safety
  void checkEncoder(void);
  //Stop motor if wrong limit activated and wrongLimitProtection is enabled
  void wrongLimitProtection(void);
  //Sets time motor has been stopped for in GalilAxis::stopped_time_
  void setStopTime(void);
  //Reset homing if stopped_time_ great than
  void checkHoming(void);
  //Service slow and infrequent requests from poll thread to write to the controller
  //We do this in a separate thread so the poll thread is not slowed
  //Also poll thread doesnt have a lock and is not allowed to call writeReadController
  void pollServices(void);
  //Execute motor record prem function
  void executePrem(void);
  //Execute auto motor power on
  bool executeAutoOn(void);
  //Execute auto motor brake off
  bool executeAutoBrakeOff(void);
  //Execute the auto on delay
  void executeAutoOnDelay(void);
  //Execute auto motor brake on
  void executeAutoBrakeOn(void);
  //Execute motor record post function
  void executePost(void);
  //Execute auto motor power off
  void executeAutoOff(void);
  //Check velocity and wlp protection
  asynStatus beginCheck(const char *functionName, double maxVelocity);
  //Begin motor motion
  asynStatus beginMotion(const char *caller);
  //Set axis brake state
  asynStatus setBrake(bool enable);
  //Restore the motor brake status after axisReady_
  asynStatus restoreBrake(void);
  //Setup home move, but dont start it
  asynStatus setupHome(double maxVelocity, int forwards);

  /* These are the methods we override from the base class */
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
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

  ~GalilAxis();

private:
  GalilController *pC_;      		/**< Pointer to the asynMotorController to which this axis belongs.
                                	*   Abbreviated because it is used very frequently */
  char axisName_;			//The axis letter A-H
  bool limit_as_home_;	     		//use limit switches for homing
  int switch_type_;			//switch type for motor enable/disable function
  char *enables_string_;		//Motor enable/disable string specified by user
  int invert_ssi_;			//Invert ssi encoder.  Reverse -ve, and +ve direction of ssi

  double highLimit_;			//High soft limit
  double lowLimit_;			//Low soft limit
  double encmratio_;			//Encoder/motor ratio
  bool encmratioset_;			//Flag to indicate if the ratio has been set
  int deferredCoordsys_;		//Coordinate system 0 (S) or 1 (T)
  double deferredAcceleration_;		//Coordinate system acceleration
  double deferredVelocity_;		//Coordinate system velocity
  double deferredPosition_;		//Deferred move position
  int deferredRelative_;		//Deferred move is relative or absolute
  bool deferredMove_;			//Has a deferred move been set
  int deferredMode_;			//Sync start and stop, or sync start only
  bool axisReady_;			//Have motor record fields been pushed into driver

  epicsTimeStamp begin_nowt_;		//Used to track length of time motor begin takes
  epicsTimeStamp begin_begint_;		//Used to track length of time motor begin takes
 
  epicsTimeStamp pestall_nowt_;		//Used to track length of time encoder has been stalled for
  epicsTimeStamp pestall_begint_;	//Time when possible encoder stall first detected
  int ueip_;				//motorRecord ueip
  int motorType_;			//MotorType set for this poll cycle
  double motor_position_;		//aux encoder or step count register
  double encoder_position_;		//main encoder register
  double last_motor_position_;		//aux encoder or step count register stored from previous poll.  Used to detect movement.
  double last_encoder_position_;	//main encoder register stored from previous poll.  Used to detect movement.
  int direction_;			//Movement direction
  bool inmotion_;			//Axis in motion status from controller
  int stop_code_;			//Axis stop code from controller
  bool fwd_;				//Forward limit status
  bool rev_;				//Reverse limit status
  bool home_;				//Home switch raw status direct from data record
  int done_;				//Motor done status passed to motor record
  int last_done_;			//Backup of done status at end of each poll.  Used to detect stop
  bool homing_;				//Is motor homing now
  epicsTimeStamp stop_nowt_;		//Used to track length of motor stopped for.
  epicsTimeStamp stop_begint_;		//Used to track length of motor stopped for.
  double stopped_time_;			//Time motor has been stopped for
  bool encDirOk_;			//Encoder direction ok flag
  bool encoderMove_;			//Encoder move status
  bool pestall_detected_;		//Possible encoder stall detected flag
  epicsMessageQueue pollRequest_;	//The service numbers poll would like done
  bool stopSent_;			//Has motor stop mesg been sent to pollServices thread due to encoder stall or wrong limit protection
  bool postSent_;			//Has post mesg been sent to pollServices thread after motor stop
  bool autooffSent_;			//Has motor auto off mesg been sent to pollServices thread after motor stop
  bool postExecuted_;			//Has pollServices executed post
  bool autooffAllowed_;			//Block autoOff and auto brake on if autoOn has released lock for on delay
  bool autobrakeonSent_;		//Auto brake on message sent to pollServices
  bool brakeInit_;			//Brake initial state
  bool homedSent_;			//Homed message sent to pollServices
  bool homedExecuted_;			//Homed message has been executed by pollServices
  bool cancelHomeSent_;			//Cancel home process message sent to pollServices

friend class GalilController;
friend class GalilCSAxis;
};

#endif   // GalilAxis_H

