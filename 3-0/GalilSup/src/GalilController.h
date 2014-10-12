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

#ifndef GalilController_H
#define GalilController_H

#include "GalilAxis.h"
#include "GalilCSAxis.h"
#include "GalilConnector.h"
#include "GalilPoller.h"

#if defined _WIN32 || _WIN64
#define rint(x) floor((x)+0.5)
#define lrint(x) floor((x)+0.5)
#define finite(x) _finite(x)
#endif /* _WIN32/_WIN64 */

#define AASCII 65
#define QASCII 81
#define SCALCARGS 16
#define MAX_GALIL_STRING_SIZE 80
#define MAX_GALIL_AXES 8
#define MAX_GALIL_VARS 10
#define MAX_GALIL_CSAXES 8
#define MAX_MESSAGE_LEN 256
#define MAX_SEGMENTS 511
#define COORDINATE_SYSTEMS 2
#define ANALOG_PORTS 8
#define BINARYIN_BYTES 7
#define BINARYOUT_WORDS 5
#define LIMIT_CODE_LEN 80000
#define INP_CODE_LEN 80000
#define THREAD_CODE_LEN 80000

// drvInfo strings for extra parameters that the Galil controller supports
#define GalilAddressString		"CONTROLLER_ADDRESS"
#define GalilHomeTypeString		"CONTROLLER_HOMETYPE"
#define GalilLimitTypeString		"CONTROLLER_LIMITTYPE"
#define GalilCtrlErrorString		"CONTROLLER_ERROR"
#define GalilCommunicationErrorString	"CONTROLLER_COMMERR"
#define GalilModelString		"CONTROLLER_MODEL"

#define GalilCoordSysString		"COORDINATE_SYSTEM"
#define GalilCoordSysMotorsString	"COORDINATE_SYSTEM_MOTORS"
#define GalilCoordSysMovingString	"COORDINATE_SYSTEM_MOVING"
#define GalilCoordSysSegmentsString	"COORDINATE_SYSTEM_SEGMENTS"
#define GalilCoordSysMotorsStopString	"COORDINATE_SYSTEM_MOTORS_STOP"
#define GalilCoordSysMotorsGoString	"COORDINATE_SYSTEM_MOTORS_GO"

#define GalilCoordSysVarString		"COORDINATE_SYSTEM_VARIABLE"

#define GalilProfileFileString		"GALIL_PROFILE_FILE"
#define GalilProfileMaxVelocityString	"GALIL_PROFILE_MAX_VELOCITY"
#define GalilProfileMaxAccelerationString	"GALIL_PROFILE_MAX_ACCELERATION"
#define GalilProfileMinPositionString	"GALIL_PROFILE_MIN_POSITION"
#define GalilProfileMaxPositionString	"GALIL_PROFILE_MAX_POSITION"
#define GalilProfileMoveModeString	"GALIL_PROFILE_MOVE_MODE"

#define GalilOutputCompare1AxisString	"OUTPUT_COMPARE_AXIS"
#define GalilOutputCompare1StartString	"OUTPUT_COMPARE_START"
#define GalilOutputCompare1IncrString	"OUTPUT_COMPARE_INCR"
#define GalilOutputCompareMessageString	"OUTPUT_COMPARE_MESSAGE"

#define GalilMotorStopGoString		"MOTOR_STOPGO"
#define GalilSSIConnectedString		"MOTOR_SSI_CONNECTED"
#define GalilEncoderStallString		"MOTOR_ENCODER_STALL"
#define GalilEncoderStallTimeString	"MOTOR_ENCODER_STALL_TIME"
#define GalilStepSmoothString		"MOTOR_STEPSMOOTH"
#define GalilEncoderDeadBString		"MOTOR_EDEL"
#define GalilMotorTypeString		"MOTOR_TYPE"
#define GalilMotorOnString		"MOTOR_ONOFF"
#define GalilMotorConnectedString	"MOTOR_MCONN"
#define GalilProgramHomeString		"MOTOR_PHOME"
#define GalilAfterLimitString		"MOTOR_EGUAFTLIMIT"
#define GalilHomeValueString		"MOTOR_HOMEVAL"
#define GalilHomedString		"MOTOR_HOMED"
#define GalilWrongLimitProtectionString	"MOTOR_WLP"
#define GalilWrongLimitProtectionActiveString	"MOTOR_WLP_ACTIVE"
#define GalilUserOffsetString		"MOTOR_OFF"
#define GalilEncoderResolutionString	"MOTOR_ERES"
#define GalilDirectionString		"MOTOR_DIR"
#define GalilUseEncoderString		"MOTOR_UEIP"
#define GalilPremString			"MOTOR_PREM"
#define GalilPostString			"MOTOR_POST"
#define GalilAutoOnOffString		"MOTOR_AUTO_ONOFF"
#define GalilAutoOnDelayString		"MOTOR_AUTO_ONDELAY"
#define GalilAutoOffDelayString		"MOTOR_AUTO_OFFDELAY"
#define GalilAutoOffFractionString	"MOTOR_AUTO_OFFFRAC"

#define GalilMainEncoderString		"MOTOR_MAIN_ENCODER"
#define GalilAuxEncoderString		"MOTOR_AUX_ENCODER"
#define GalilMotorAcclString		"MOTOR_ACCL"
#define GalilMotorVeloString		"MOTOR_VELO"
#define GalilMotorVmaxString		"MOTOR_VMAX"
#define GalilAnalogInString		"ANALOG_IN"
#define GalilAnalogOutString		"ANALOG_OUT"
#define GalilAnalogOutRBVString		"ANALOG_OUTRBV"
#define GalilBinaryInString		"BINARY_IN"
#define GalilBinaryOutString		"BINARY_OUT"
#define GalilBinaryOutRBVString		"BINARY_OUTRBV"
#define GalilStopEventString		"CONTROLLER_STOPE"
#define GalilSSICapableString		"CONTROLLER_SSICAPABLE"
#define GalilSSIInputString		"MOTOR_SSIINPUT"
#define GalilSSITotalBitsString		"MOTOR_SSITOTBITS"
#define GalilSSISingleTurnBitsString	"MOTOR_SSISINGLETBITS"
#define GalilSSIErrorBitsString		"MOTOR_SSIERRBITS"
#define GalilSSITimeString		"MOTOR_SSITIME"
#define GalilSSIDataString		"MOTOR_SSIDATA"
#define GalilErrorLimitString		"MOTOR_ERRLIM"
#define GalilErrorString		"MOTOR_ERR"
#define GalilOffOnErrorString		"MOTOR_OOE"
#define GalilAxisString			"MOTOR_AXIS"

#define GalilUserCmdString		"USER_CMD"
#define GalilUserOctetString		"USER_OCTET"
#define GalilUserOctetValString		"USER_OCTET_VAL"
#define GalilUserVarString		"USER_VAR"

/* For each digital input, we maintain a list of motors, and the state the input should be in*/
/* To disable the motor */
struct Galilmotor_enables {
	char motors[MAX_GALIL_AXES];
	char disablestates[MAX_GALIL_AXES];
};

class GalilController : public asynMotorController {
public:
  GalilController(const char *portName, const char *address, double updatePeriod);
  
  /* These are the methods that we override from asynMotorController */
  asynStatus poll(void);
  asynStatus setDeferredMoves(bool deferMoves);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeOctet(asynUser *pasynUser, const char*  value,  size_t  nChars,  size_t *  nActual);
  asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
  asynStatus drvUserCreate(asynUser *pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize); 
  asynStatus drvUserDestroy(asynUser *pasynUser);
  void report(FILE *fp, int level);

  //Real motors
  GalilAxis* getAxis(asynUser *pasynUser);
  GalilAxis* getAxis(int axisNo);

  //Coordinate system axis
  GalilCSAxis* getCSAxis(asynUser *pasynUser);
  GalilCSAxis* getCSAxis(int axisNo);
  asynStatus breakupTransform(char *raw, char *axes, char **equations);
  asynStatus findVariableSubstitutes(char *axes, char *csaxes, char **equations, char **variables, char **substitutes);

  /* These are the methods that we override from asynPortDriver */
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);

  /* These are the functions for profile moves */
  //asynStatus initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword);
  asynStatus buildProfile();
  asynStatus buildLinearProfile();
  asynStatus executeProfile();
  asynStatus abortProfile();
  //asynStatus readbackProfile();

  //Execute motor record prem function for motor list
  void executePrem(const char *axes);

  //Execute motor power auto on
  void executeAutoOn(const char *axes);

  /* These are the methods that are new to this class */
  void GalilStartController(char *code_file, int eeprom_write, int display_code);
  void connectManager(void);
  void connect(void);
  void connected(void);
  asynStatus acquireDataRecord(string cmd);
  void getStatus(void);
  void setParamDefaults(void);
  void gen_card_codeend(void);
  void gen_motor_enables_code(void);
  void write_gen_codefile(void);
  void read_codefile(const char *code_file);
  asynStatus writeReadController(const char *caller);
  void check_comms(bool reqd_comms, asynStatus status);
  asynStatus get_integer(int function, epicsInt32 *value, int axisNo);
  asynStatus get_double(int function, epicsFloat64 *value, int axisNo);
  void profileThread();
  asynStatus setOutputCompare(int oc);
  asynStatus runProfile();
  asynStatus runLinearProfile(FILE *profFile);
  bool motorsMoving(char *axes);
  asynStatus startLinearProfileCoordsys(char coordName, const char *axes);
  asynStatus motorsToProfileStartPosition(FILE *profFile, char *axes, bool move);

  /* Deferred moves functions.*/
  asynStatus processDeferredMovesInGroup(int coordsys, char *axes, char *moves, double acceleration, double velocity);

protected:
  #define FIRST_GALIL_PARAM GalilAddress_
  int GalilAddress_;
  int GalilHomeType_;
  int GalilLimitType_;
  int GalilCtrlError_;
  int GalilCoordSys_;
  int GalilCoordSysMotors_;
  int GalilCoordSysMoving_;
  int GalilCoordSysSegments_;
  int GalilCoordSysMotorsStop_;
  int GalilCoordSysMotorsGo_;
  int GalilCoordSysVar_;
  int GalilProfileFile_;
  int GalilProfileMaxVelocity_;
  int GalilProfileMaxAcceleration_;
  int GalilProfileMinPosition_;
  int GalilProfileMaxPosition_;
  int GalilProfileMoveMode_;

  int GalilOutputCompareAxis_;
  int GalilOutputCompareStart_;
  int GalilOutputCompareIncr_;
  int GalilOutputCompareMessage_;

  int GalilMotorStopGo_;
  int GalilEStall_;
  int GalilEStallTime_;
  int GalilStepSmooth_;
  int GalilEncoderDeadB_;
  int GalilModel_;
  int GalilMotorType_;
  int GalilMotorOn_;
  int GalilMotorConnected_;
  int GalilProgramHome_;
  int GalilAfterLimit_;
  int GalilHomeValue_;
  int GalilHomed_;
  int GalilWrongLimitProtection_;
  int GalilWrongLimitProtectionActive_;
  int GalilUserOffset_;
  int GalilEncoderResolution_;
  int GalilUseEncoder_;
  int GalilPrem_;
  int GalilPost_;
  int GalilAutoOnOff_;
  int GalilAutoOnDelay_;
  int GalilAutoOffDelay_;
  int GalilAutoOffFraction_;

  int GalilMainEncoder_;
  int GalilAuxEncoder_;
  int GalilMotorAccl_;
  int GalilMotorVelo_;
  int GalilMotorVmax_;
  int GalilAnalogIn_;
  int GalilAnalogOut_;
  int GalilAnalogOutRBV_;
  int GalilBinaryIn_;
  int GalilBinaryOut_;
  int GalilBinaryOutRBV_;
  int GalilStopEvent_;
  int GalilDirection_;
  int GalilSSIConnected_;
  int GalilSSICapable_;
  int GalilSSIInput_;
  int GalilSSITotalBits_;
  int GalilSSISingleTurnBits_;
  int GalilSSIErrorBits_;
  int GalilSSITime_;
  int GalilSSIData_;
  int GalilErrorLimit_;
  int GalilError_;
  int GalilOffOnError_;
  int GalilAxis_;
  int GalilUserCmd_;
  int GalilUserOctet_;
  int GalilUserOctetVal_;
  int GalilUserVar_;
//Add new parameters here

  int GalilCommunicationError_;
  #define LAST_GALIL_PARAM GalilCommunicationError_

private:
  asynUser *pasynUserGalil_;
  Galil *gco_;				//Galil communication object (gco_).  From galil communication lib
  GalilPoller *poller_;			//GalilPoller to acquire a datarecord
  char address_[256];			//address string
  char model_[256];			//model string
  char code_file_[256];			//Code file that user gave to GalilStartController
  int eeprom_write_;			//eeprom_write_ that user gave to GalilStartController
  bool connect_fail_reported_;		//Has initial connection failure been reported to iocShell
  int consecutive_timeouts_;		//Used for connection management
  bool code_assembled_;			//Has code for the GalilController hardware been assembled
  bool async_records_;			//Are the data records obtained async(DR), or sync (QR)
  double updatePeriod_;			//Period between data records in ms

  epicsTimeStamp pollnowt_;		//Used for debugging, and tracking overall poll performance
  epicsTimeStamp polllastt_;		//Used for debugging, and tracking overall poll performance

  bool movesDeferred_;			//Should moves be deferred for this controller

  epicsEventId profileExecuteEvent_;	//Event for executing motion profiles
  bool profileAbort_;			//Abort profile request flag.  Aborts profile when set true

  vector<char> recdata_;		//Data record from controller
  asynStatus recstatus_;		//Status of last record acquisition
  unsigned numAxesMax_;			//Number of axes actually supported by the controller
  unsigned numAxes_;			//Number of axes requested by developer
  unsigned numThreads_;			//Number of threads the controller supports
  bool codegen_init_;			//Has the code generator been initialised for this controller
  bool digitalinput_init_;		//Has the digital input label #ININT been included for this controller
  char *thread_code_;			//Code generated for every axis on this controller (eg. home code, stepper pos maintenance)
  char *limit_code_;			//Code generated for limit switches on this controller
  char *digital_code_;			//Code generated for digital inputs on this controller
  char *card_code_;			//All code generated for the controller

  char cmd_[MAX_GALIL_STRING_SIZE];     //holds the assembled Galil cmd string
  char resp_[MAX_GALIL_STRING_SIZE];    //Response from Galil controller

					//Stores the motor enable disable interlock digital IO setup, only first 8 digital in ports supported
  struct Galilmotor_enables motor_enables_[8];

  friend class GalilAxis;
  friend class GalilCSAxis;
  friend class GalilPoller;
};
#define NUM_GALIL_PARAMS (&LAST_GALIL_PARAM - &FIRST_GALIL_PARAM + 1)
#endif  // GalilController_H
