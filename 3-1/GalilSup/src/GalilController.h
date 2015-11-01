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

#if defined _WIN32 || _WIN64
#define rint(x) floor((x)+0.5)
#define lrint(x) floor((x)+0.5)
#define finite(x) _finite(x)
#endif /* _WIN32/_WIN64 */

#define BEGIN_TIMEOUT 2.5
#define AASCII 65
#define IASCII 73
#define QASCII 81
#define SCALCARGS 16
//Number of communication retries
#define ALLOWED_TIMEOUTS 2
#define MAX_GALIL_UNSOLICTED_SIZE 29
#define MAX_GALIL_STRING_SIZE 256
#define MAX_GALIL_DATAREC_SIZE 512
#define MAX_GALIL_AXES 8
#define MAX_GALIL_VARS 10
#define MAX_GALIL_CSAXES 8
#define MAX_FILENAME_LEN 2048
#define MAX_SEGMENTS 511
#define COORDINATE_SYSTEMS 2
#define ANALOG_PORTS 8
#define BINARYIN_BYTES 7
#define BINARYOUT_WORDS 5
#define LIMIT_CODE_LEN 80000
#define INP_CODE_LEN 80000
#define THREAD_CODE_LEN 80000
#define CODE_LENGTH 80000

#include "macLib.h"
#include "GalilAxis.h"
#include "GalilCSAxis.h"
#include "GalilConnector.h"
#include "GalilPoller.h"
#include "epicsMessageQueue.h"

#include <unordered_map> //used for data record features
#include <vector>

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

#define GalilCSMotorVariableString	"CSMOTOR_KINEMATIC_VARIABLE"
#define GalilCSMotorForwardString	"CSMOTOR_FORWARD_TRANSFORM"
#define GalilCSMotorReverseAString	"CSMOTOR_REVERSEA_TRANSFORM"
#define GalilCSMotorReverseBString	"CSMOTOR_REVERSEB_TRANSFORM"
#define GalilCSMotorReverseCString	"CSMOTOR_REVERSEC_TRANSFORM"
#define GalilCSMotorReverseDString	"CSMOTOR_REVERSED_TRANSFORM"
#define GalilCSMotorReverseEString	"CSMOTOR_REVERSEE_TRANSFORM"
#define GalilCSMotorReverseFString	"CSMOTOR_REVERSEF_TRANSFORM"
#define GalilCSMotorReverseGString	"CSMOTOR_REVERSEG_TRANSFORM"
#define GalilCSMotorReverseHString	"CSMOTOR_REVERSEH_TRANSFORM"

#define GalilMotorStopGoString		"MOTOR_STOPGO"
#define GalilSSIConnectedString		"MOTOR_SSI_CONNECTED"
#define GalilEncoderStallString		"MOTOR_ENCODER_STALL"
#define GalilEncoderStallTimeString	"MOTOR_ENCODER_STALL_TIME"
#define GalilStepSmoothString		"MOTOR_STEPSMOOTH"
#define GalilEncoderDeadBString		"MOTOR_EDEL"
#define GalilMotorTypeString		"MOTOR_TYPE"
#define GalilMotorOnString		"MOTOR_ONOFF"
#define GalilMotorConnectedString	"MOTOR_MCONN"
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
#define GalilUseIndexString		"MOTOR_USEINDEX"
#define GalilJogAfterHomeString		"MOTOR_JOG_AHOME"
#define GalilJogAfterHomeValueString	"MOTOR_JOG_AHOME_VALUE"
#define GalilAutoOnOffString		"MOTOR_AUTO_ONOFF"
#define GalilAutoOnDelayString		"MOTOR_AUTO_ONDELAY"
#define GalilAutoOffDelayString		"MOTOR_AUTO_OFFDELAY"
#define GalilAutoBrakeString		"MOTOR_AUTO_BRAKE"
#define GalilAutoBrakeOnDelayString     "MOTOR_AUTO_BRAKEONDELAY"
#define GalilBrakePortString		"MOTOR_BRAKEPORT"
#define GalilBrakeString		"MOTOR_BRAKE"

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
#define GalilMotorVelocityEGUString  	"MOTOR_VELOCITY_EGU"
#define GalilMotorVelocityRAWString  	"MOTOR_VELOCITY_RAW"

#define GalilUserCmdString		"USER_CMD"
#define GalilUserOctetString		"USER_OCTET"
#define GalilUserOctetValString		"USER_OCTET_VAL"
#define GalilUserVarString		"USER_VAR"

#define GalilEthAddrString	  	"CONTROLLER_ETHADDR"
#define GalilSerialNumString	  	"CONTROLLER_SERIALNUM"

/* For each digital input, we maintain a list of motors, and the state the input should be in*/
/* To disable the motor */
struct Galilmotor_enables {
	char motors[MAX_GALIL_AXES];
	char disablestates[MAX_GALIL_AXES];
};

struct Source //each data record source key (e.g. "_RPA") maps to one of these, each of which describes the position and width of the variable within the binary data record
{
	int byte; //byte offset within binary data record
	std::string type; //"SB", "UB", "SW", "UW", "SL", "UL".  Specifies width within binary data record and signed/unsigned.
	int bit; //-1 if not bit field (e.g. RPA).  >= 0 if bit field (e.g. _MOA)
	std::string units; //e.g. "counts"
	std::string description; //e.g. "analog input 1"
	double scale; //e.g. 32768, scale factor:  most sources are 1 except TV, TT, @AN, @AO etc.
	double offset; //needed for analog inputs and outputs

	Source(int byte = 0, std::string type = "Ux", int bit = -1, std::string units = "", std::string description = "", double scale = 1, double offset = 0) :
		byte(byte), type(type), bit(bit), units(units), description(description), scale(scale), offset(offset)
	{ /*ctor just initializes values*/ }
};

class GalilController : public asynMotorController {
public:
  //These variables need to be accessible from static callbacks
  epicsEventId connectEvent_;		//Connection event
  int connected_;			//Is the synchronous communication socket connected according to asyn.  Async UDP is connectionless

  //Class constructor
  GalilController(const char *portName, const char *address, double updatePeriod);

  asynStatus async_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
  asynStatus async_writeReadController(void);

  asynStatus sync_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
  asynStatus sync_writeReadController(void);

  asynStatus sendUnsolicitedMessage(char *mesg);
  bool my_isascii(int c);
  asynStatus programUpload(string *prog);
  asynStatus programDownload(string prog);
  
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

  /* These are the methods that we override from asynPortDriver */
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);

  /* These are the functions for profile moves */
  //asynStatus initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword);
  asynStatus buildProfile();
  asynStatus buildLinearProfile();
  asynStatus executeProfile();
  asynStatus abortProfile();
  //asynStatus readbackProfile();

  /* These are the methods that are new to this class */
  void GalilStartController(char *code_file, int eeprom_write, int display_code, unsigned thread_mask);
  void connect(void);
  void disconnect(void);
  void connected(void);
  void acquireDataRecord(string cmd);
  asynStatus readDataRecord(asynUser *pasynUser, char *input, unsigned bytesize);
  void getStatus(void);
  void setParamDefaults(void);
  void gen_card_codeend(void);
  void gen_motor_enables_code(void);
  void write_gen_codefile(const char* suffix);
  asynStatus read_codefile(const char *code_file);
  asynStatus read_codefile_part(const char *code_file, MAC_HANDLE* mac_handle);
  void check_comms(bool reqd_comms, asynStatus status);
  asynStatus get_integer(int function, epicsInt32 *value, int axisNo);
  asynStatus get_double(int function, epicsFloat64 *value, int axisNo);
  void profileThread();
  asynStatus setOutputCompare(int oc);
  asynStatus runProfile();
  asynStatus runLinearProfile(FILE *profFile);
  bool motorsMoving(char *axes);
  asynStatus startLinearProfileCoordsys(int coordsys, char coordName, const char *axes);
  asynStatus motorsToProfileStartPosition(FILE *profFile, char *axes, bool move);
  //Execute motor record prem function for motor list
  void executePrem(const char *axes);
  //Execute auto motor power on, and brake off 
  void executeAutoOnBrakeOff(const char *axes);
  void processUnsolicitedMesgs(void);
  static std::string extractEthAddr(const char* str);
  void setCtrlError(const char* mesg);

  void InitializeDataRecord(void);
  double sourceValue(const std::vector<char>& record, const std::string& source);
  void Init30010(bool dmc31010);
  void Init4000(int axes);
  void Init2103(int axes);
  void InitRio(bool rio3);
  void InitRio3_24Ex(void);
  void InitRioSer(bool rio3);
  void aq_analog(int byte, int input_num);
  string ax(string prefix, int axis, string suffix);
  void input_bits(int byte, int num);
  void output_bits(int byte, int num);
  void dq_analog(int byte, int input_num);

  /* Deferred moves functions.*/
  asynStatus processDeferredMovesInGroup(int coordsys, char *axes, char *moves, double acceleration, double velocity);

  void shutdownController();
  ~GalilController();

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

  int GalilCSMotorVariable_;
  int GalilCSMotorForward_;
  int GalilCSMotorReverseA_;
  int GalilCSMotorReverseB_;
  int GalilCSMotorReverseC_;
  int GalilCSMotorReverseD_;
  int GalilCSMotorReverseE_;
  int GalilCSMotorReverseF_;
  int GalilCSMotorReverseG_;
  int GalilCSMotorReverseH_;

  int GalilMotorStopGo_;
  int GalilEStall_;
  int GalilEStallTime_;
  int GalilStepSmooth_;
  int GalilEncoderDeadB_;
  int GalilModel_;
  int GalilMotorType_;
  int GalilMotorOn_;
  int GalilMotorConnected_;
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
  int GalilUseIndex_;
  int GalilJogAfterHome_;
  int GalilJogAfterHomeValue_;
  int GalilAutoOnOff_;
  int GalilAutoOnDelay_;
  int GalilAutoOffDelay_;
  int GalilAutoBrake_;
  int GalilAutoBrakeOnDelay_;
  int GalilBrakePort_;
  int GalilBrake_;

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
  int GalilMotorVelocityEGU_;
  int GalilMotorVelocityRAW_;
  int GalilUserCmd_;
  int GalilUserOctet_;
  int GalilUserOctetVal_;
  int GalilUserVar_;
  int GalilEthAddr_;
  int GalilSerialNum_;
//Add new parameters here

  int GalilCommunicationError_;
  #define LAST_GALIL_PARAM GalilCommunicationError_

private:

  std::unordered_map<std::string, Source> map; //data structure for data record

  char cmd_[MAX_GALIL_STRING_SIZE];	//holds the assembled Galil cmd string
  char resp_[MAX_GALIL_DATAREC_SIZE];	//Response from Galil controller

  GalilPoller *poller_;			//GalilPoller to acquire a datarecord
  GalilConnector *connector_;		//GalilConnector to manage connection status flags

  char address_[MAX_GALIL_STRING_SIZE];	//address string
  char model_[MAX_GALIL_STRING_SIZE];	//model string
  bool rio_;				//Is controller a RIO
  char code_file_[MAX_FILENAME_LEN];	//Code file(s) that user gave to GalilStartController

  int burn_program_;			//Burn program options that user gave to GalilStartController
					
  int consecutive_timeouts_;		//Used for connection management
  bool code_assembled_;			//Has code for the GalilController hardware been assembled (ie. is card_code_ all set to send)
  double updatePeriod_;			//Period between data records in ms
  bool async_records_;			//Are the data records obtained async(DR), or sync (QR)
  bool try_async_;			//Should we even try async udp (DR) before going to synchronous tcp (QR) mode

  epicsTimeStamp begin_nowt_;		//Used to track length of time motor begin takes
  epicsTimeStamp begin_begint_;		//Used to track length of time motor begin takes

  bool movesDeferred_;			//Should moves be deferred for this controller

  bool coordSysStopping_[2];		//Coordinate system stopping status.  Used to process limit status for csaxes

  epicsEventId profileExecuteEvent_;	//Event for executing motion profiles
  bool profileAbort_;			//Abort profile request flag.  Aborts profile when set true
  unsigned thread_mask_;		//Mask detailing which threads are expected to be running after program download Bit 0 = thread 0 etc

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
  char *card_code_;			//All code generated for the controller.  This is the buffer actually sent to controller
  char *user_code_;			//Code supplied by user for the controller.  This is copied to card_code_ above if all goes well

  char asynccmd_[MAX_GALIL_STRING_SIZE];	//holds the assembled Galil cmd string
  char asyncresp_[MAX_GALIL_DATAREC_SIZE];	//For asynchronous messages including datarecord

  int timeout_;				//Timeout for communications
  int controller_number_;		//The controller number as counted in GalilCreateController
  epicsMessageQueue unsolicitedQueue_;	//Unsolicted messages recieved are placed here initially
  
  char syncPort_[MAX_GALIL_STRING_SIZE];	//The name of the asynPort created for synchronous communication with controller
  char asyncPort_[MAX_GALIL_STRING_SIZE];	//The name of the asynPort created for asynchronous communication with controller
  char udpHandle_;				//Handle on controller used for udp
  char syncHandle_;				//Handle on controller used for synchronous communication (ie. tcp or serial)
  unsigned datarecsize_;			//Calculated size of controller datarecord based on response from QZ command
  asynUser *pasynUserSyncGalil_;		//Asyn user for synchronous communication
  asynCommon *pasynCommon_;			//asynCommon interface for synchronous communication
  void *pcommonPvt_;				//asynCommon drvPvt for synchronous communication
  asynUser *pasynUserAsyncGalil_;		//Asyn user for asynchronous communication

  struct Galilmotor_enables motor_enables_[MAX_GALIL_AXES];//Stores the motor enable disable interlock digital IO setup, only first 8 digital in ports supported

  friend class GalilAxis;
  friend class GalilCSAxis;
  friend class GalilPoller;
  friend class GalilConnector;
};
#define NUM_GALIL_PARAMS (&LAST_GALIL_PARAM - &FIRST_GALIL_PARAM + 1)
#endif  // GalilController_H
