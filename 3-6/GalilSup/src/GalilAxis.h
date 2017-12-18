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

//Stop codes
//These are Galil Stop codes
#define MOTOR_STOP_FWD 2
#define MOTOR_STOP_REV 3
#define MOTOR_STOP_STOP 4
#define MOTOR_STOP_ONERR 8
#define MOTOR_STOP_ENC 12
#define MOTOR_STOP_AMP 15
#define MOTOR_STOP_ECATCOMM 70
#define MOTOR_STOP_ECATAMP 71
//Our custom codes
#define MOTOR_OKAY 0
#define MOTOR_STOP_ONWLP 255
#define MOTOR_STOP_ONSTALL 256

//pollServices request numbers
static const int MOTOR_STOP = 0;
static const int MOTOR_POST = 1;
static const int MOTOR_OFF = 2;
static const int MOTOR_HOMED = 3;
static const int MOTOR_CANCEL_HOME = 4;
static const int MOTOR_BRAKE_ON = 5;
static const int MOTOR_STEP_SYNC_ATSTOP = 6;
static const int MOTOR_STEP_SYNC_ATENC = 7;

//BISS constants
const int BISS_INPUT_MIN = 0;
const int BISS_INPUT_MAX = 2;
const int BISS_DATA1_MIN = -38;
const int BISS_DATA1_MAX = 38;
const int BISS_DATA2_MIN = 0;
const int BISS_DATA2_MAX = 38;
const int BISS_ZP_MIN = 0;
const int BISS_ZP_MAX = 7;
const int BISS_CD_MIN = 4;
const int BISS_CD_MAX = 26;

const epicsUInt32 BISS_STAT_TIMEOUT = 0;
const epicsUInt32 BISS_STAT_CRC =     1;
const epicsUInt32 BISS_STAT_ERROR =   2;
const epicsUInt32 BISS_STAT_WARN =    3;

//limitState enum used in wrong limit protection
typedef enum limitsState
   {
   unknown, consistent, not_consistent
   } limitsState;

class GalilAxis : public asynMotorAxis
{
public:

  GalilAxis(class GalilController *pC, 
	    char *axisname,			/*axisname A-H */
	    int limit_as_home,			/*0=no, 1=yes. Using a limit switch as home*/
	    char *enables_string,		/*digital input(s) to use to send home, or interlock motor if -ve*/
	    int switch_type);			/*digital input to use to send away from home*/

  //These are the methods that are new to this class
  //Poller for axis
  asynStatus poller(void);

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
  //Invert SSI encoder direction
  asynStatus invert_ssi(void);
  //Set BISS setting on controller
  asynStatus set_biss(void);
  //Get BISS setting from controller
  asynStatus get_biss(int function, epicsInt32 *value);
  //Set acceleration and velocity
  asynStatus setAccelVelocity(double acceleration, double velocity, bool setVelocity = true);
  //Extract axis data from GalilController data record
  asynStatus getStatus(void);
  //Set poller status variables bassed on GalilController data record info
  void setStatus(bool *moving);
  //Verify encoder operation whilst moving for safety
  void checkEncoder(void);
  //Synchronize aux register with encoder
  void syncEncodedStepper(void);
  //Stop motor if wrong limit activated and wrongLimitProtection is enabled
  void wrongLimitProtection(void);
  //Sets time motor has been stopped for in GalilAxis::stoppedTime_
  void setStopTime(void);
  //Reset homing if stoppedTime_ greater than HOMING_TIMEOUT
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
  asynStatus beginCheck(const char *functionName, double maxVelocity, bool resetCtrlMessage = true);
  //Begin motor motion
  asynStatus beginMotion(const char *caller, double position = 0.0, bool relative = false, bool checkpos = false, bool move = true);
  //Set axis brake state
  asynStatus setBrake(bool enable);
  //Restore the motor brake status after axisReady_
  asynStatus restoreBrake(void);
  //Setup home move, but dont start it
  asynStatus setupHome(double maxVelocity, int forwards);
  //Copy profileBackupPositions_ back into profilePositions_ after a CSAxis profile has been built
  void restoreProfileData(void);
  //Check motor record status for this axis
  asynStatus checkMRSettings(bool moveVelocity, char callaxis);
  //Tell this axis to use CSAxis dynamics
  void setCSADynamics(double acceleration, double velocity);
  //Send move to this motor via the motor record
  asynStatus moveThruMotorRecord(double position);
  //Driver internal version of axis stop, prevents backlash, retries till dmov
  asynStatus stopInternal(double acceleration);
  //Check the BiSS encoder status if it's enabled
  asynStatus checkBISSStatus(void);
  asynStatus checkBISSStatusService(void);
  //Thread function for polling any axis or encoder status that is not part of the data record 
  void axisStatusThread();
  //Clear axis EtherCat fault
  asynStatus clearEtherCatFault();

  /* These are the methods we override from the base class */
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus syncPosition(void);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setEncoderRatio(double ratio);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus initializeProfile(size_t maxProfilePoints);

  virtual ~GalilAxis();

private:

  void axisStatusShutdown(void);        //Function to shutdown axis status thread.

  GalilController *pC_;      		/**< Pointer to the asynMotorController to which this axis belongs.
                                	*   Abbreviated because it is used very frequently */
  char axisName_;			//The axis letter A-H
  bool limit_as_home_;	     		//use limit switches for homing
  int switch_type_;			//switch type for motor enable/disable function
  char *enables_string_;		//Motor enable/disable string specified by user
  bool invert_ssi_;			//Invert ssi encoder.  Reverse -ve, and +ve direction of ssi

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

  double limdc_;			//Deceleration on limit active
  double userDataDeadb_;		//User data deadband value
  double userDataPosted_;		//User data posted to upper layers

  epicsTimeStamp begin_nowt_;		//Used to track length of time motor begin takes
  epicsTimeStamp begin_begint_;		//Used to track length of time motor begin takes
 
  epicsTimeStamp pestall_nowt_;		//Used to track length of time encoder has been stalled for
  epicsTimeStamp pestall_begint_;	//Time when possible encoder stall first detected

  int ueip_;				//motorRecord ueip.  User wants to read main encoder if true, aux if false
  bool ctrlUseMain_;			//Based on selected motor type controller will use main or aux encoder register for positioning
  double motor_position_;		//aux encoder or step count register
  double encoder_position_;		//main encoder register
  double last_encoder_position_;	//main encoder register stored from previous poll.  Used to detect movement.
  double velocity_;			//Motor velocity readback
  double error_;			//Position error readback
  int direction_;			//Movement direction
  bool inmotion_;			//Axis in motion status from controller
  int stop_code_;			//Axis stop code from controller
  bool fwd_;				//Forward limit status
  bool rev_;				//Reverse limit status

  double csaAcceleration_;		//CSAxis requested acceleration
  double csaVelocity_;			//CSAxis requested velocity
  bool useCSADynamics_;			//Should this axis use CSAxis requested acceleration and velocity

  bool startDeferredMoves_;		//Used to start deferred moves automatically
  
  limitsState limitsDirState_;		//Status of limits consistency with motor direction
  bool beginOnLimit_;			//Did move begin while on a limit switch
  bool home_;				//Home switch raw status direct from data record
  int done_;				//Motor done status passed to motor record
  int last_done_;			//Backup of done status at end of each poll.  Used to detect stop
  bool homing_;				//Is motor homing now
  bool jogAfterHome_;			//Is motor doing jah
  bool stop_axis_;			//Used to prevent retries after stop
  int stop_reason_;			//Reason axis stop requested
  epicsTimeStamp stop_nowt_;		//Used to track length of time motor stopped for.
  epicsTimeStamp stop_begint_;		//Used to track length of time motor stopped for.
  double stoppedTime_;			//Time motor has been stopped for
  bool resetStoppedTime_;		//Request poll thread reset stopped time if done true
  epicsEventId stoppedTimeResetEventId_;//Signal that poller has completed reset stop time request
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
  bool syncEncodedStepperAtStopSent_;	//Synchronize stepper with encoder at stop message sent to pollServices
  bool syncEncodedStepperAtStopExecuted_;//Synchronize stepper with encoder at stop execution complete
  bool syncEncodedStepperAtEncSent_;	//Synchronize stepper with encoder at encoder move message sent
  bool encoderSwapped_;			//Have the main, and auxiliary encoders been swapped by DFx=1

  bool restoreProfile_;			//Should profileBackupPositions_ be copied into profilePositions_ after orofile built complete? 
                                	//True for all GalilAxis involved in CSAxis profile build, set false at built end
  double *profileBackupPositions_;	//Profile positions backup for this axis, restored after profile is built

  bool axisStatusShutdown_;		//Flag to shutdown axis status thread
  bool axisStatusRunning_;		//Flag to indicate if axis status thread is running
  epicsEventId axisStatusShutdownId_;	//Shutdown signal for axis status thread

friend class GalilController;
friend class GalilCSAxis;
};

#endif   // GalilAxis_H

