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

// Change log:
// 16/09/14 M.Clift First release
// 25/09/14 F.Akeroyd ISIS UK Added drvUserCreate/Destroy
// 25/09/14 F.Akeroyd ISIS UK Added writeOctet
// 25/09/14 F.Akeroyd ISIS UK Repaired some windows build issues
// 29/09/14 M.Clift Modified writeOctet drvUserCreate/Destroy
//                  Modified userdef records
// 09/10/14 M.Clift Repaired some more windows build issues
// 09/10/14 M.Clift Repaired gcl problems under windows
// 10/10/14 M.Clift Added Motor record PREM/POST support
// 11/10/14 M.Clift & F.Akeroyd Added auto pwr on/off features
// 02/11/14 F.Akeroyd Added ability to construct galil code from template
// 02/11/14 F.Akeroyd & M.Clift enhanced GalilStartController handling of user code
// 02/11/14 M.Clift Changed homing mechanism to use unsolicted messaging
// 02/11/14 M.Clift Several bug fixes in autooff, GalilCSAxis, polling
// 09/11/14 M.Clift Re-named drive after home to Jog after home
//                  Moved Jog after home, and program home register functionality into driver.
//                  Fixed bugs in homing
//                  Block moves until necessary motor record fields have been pushed into driver
// 12/11/14 M.Clift Moved reset homing on soft limit functionality into driver
//                  Fixed bugs in homing
//                  Re-wrote galil limit code in code generator with 30% improvement in efficiency
//                  Re-wrote galil home code in code generator to improve robustness
//                  Program home registers now always on
//                  Homing now uses HVEL also for home switch jog off rather than HVEL/10
// 12/11/14 F.Akeroyd Added reading of MAC address at connect
// 23/11/14 F.Akeroyd Cleanup of shutdown code, added setCtrlError and forced sync poll mode
// 26/11/14 M.Clift CSAxis now uses dial coordinates for transforms so different resolutions can be used
//                  Modified galil limits code to cope with CSAxis hitting limits
//                  Modified CSAxis limit reporting logic
// 08/01/15 M.Clift Enhanced kinematics so that transform equations can be changed via database
//                  Real motors are now allowed in reverse transform equations
//                  CSAxis are now allowed in forward transform equations
//                  Enhanced shutdown code to delete RAM used by kinematics
// 05/04/15 M.Clift Added ability to actuate motor brake using digital output
//                  Added state names and alarm severity to digital template/substitutions
//                  Fixed problem with CSAxis setpoint at startup
//                  Fixed problem with CSAxis using AutoOff delay at move start.  It now uses AutoOn delay at move start
//                  Fixed problem with config synApps build directory
// 05/05/15 M.Clift, D.J.Roberts, T.Miller Galil Motion Control
//                  Added source code for data record decoding
//                  Replaced gcl communications with Asyn communications, and removed all gcl references
// 30/10/15 M.Clift
//                  Fixed encoder stall when using deferred moves
//                  Fixed SSI capability detection on 4xxx controllers
//                  Fixed analog numbering for DMC controllers
//                  Fixed threading issue in poller caused by motorUpdateStatus_
//                  Fixed mixed open and closed loop motor in CSAxis
//                  Fixed command console
//                  Fixed coordsys selection problem in deferred moves
//                  CSaxis can now be used in deferred moves
// 3/11/15 M.Clift
//                  Reverse transform now allows multiple new position setpoints at the same time
//                  Deferred move now allows multiple related CSAxis to be moved at the same time
// 9/11/15 M.Clift
//                  Removed encoder deadband from motor extras
//                  Added motor record retry deadband
//                  Improved messaging for profile moves
// 17/11/15 M.Clift 
//                  Add velocity, acceleration transforms to CSAxis
//                  Add velocity checking to CSAxis
// 23/11/15 M.Clift 
//                  Add deferredMode supporting Sync motor start only, and Sync motor start and stop
//                  Fixed multiple related CSAxis deferred moves were being lost
// 28/01/16 M.Clift
//                  Fix SSI capability detection on DMC4xxx series
//                  Fix SSI encoder connected flag issue when encoder connected to auxillary input
// 28/01/16 E.Norum & M.Clift
//                  Fix thread count issue with DMC30000
//                  Fix problem with DMC30000 not returning \r\n when uploading program
//                  Fix problem with DMC30000 datarecord decoding
//                  Revised setOutputCompare to make compatible with DMC30000
// 31/01/16 M.Clift
//                  Fix seg fault on exit when no controllers are connected
//                  Fix issue where SSI input setting for motor A effected all motors
//                  Fix galil gmc code stop caused by CSAxis stop call when no revaxes assigned
//                  Simplified communications checking
//                  Simplified CSAxis limit reporting
//                  Fix CSAxis jog motor run away in Sync start only mode when limit struck
// 31/01/16 K.Paterson
//                  Fix seg fault on startup because of RIO-47300-16BIT-24EXOUT large reply
// 11/02/16 M.Clift 
//                  Tidy up CSAxis reverse, forward transforms
//                  Further fixes to setOutputCompare
//                  Added writeFloat64Array interface
//                  CSAxis can now be used in profile motion
//                  Added parameter profile type. Linear or PVT
// 22/02/16 M.Clift
//                  Added home allowed to motor extras
//                  Reworked MEDM screens for APS fonts
//                  Prevent loss of GalilAxis profile data when using GalilCSAxis in profiles
//                  Add PVT capable detection code
//                  Add motor home allowed (eg. None, reverse, forward, both) setting
//                  Change buildLinearProfile to buildProfileFile
//                  buildProfileFile can now build linear or pvt profiles
// 26/02/16 M.Clift 
//                  Add CSAxis home method
//                  Consolidation and tidy up
//                  Further minor adjustments to Qt and MEDM screens
//                  Minor adjustments to begin motion methods
// 28/02/10 M.Clift 
//                  Fixed problem in vector calculations
//                  Fixed problem with motor interlock function
// 08/03/16 M.Clift
//                  Add PVT profile ability completed
// 14/03/16 M.Clift
//                  Fixed problem with async connection terminator during udp handle discovery
//                  Rewrote readDataRecord resulting in upto 10% reduced CPU load
//                  Fixed issue with datarecord header on some models
//                  Adjustments to profile buffering
//                  Remove counter from generated code
// 17/03/16 M.Davis & M.Clift
//                  Fix problem with extractEthAddr string length
// 20/03/16 M.Clift
//                  Added analog readback deadbands
//                  Added PWM servo motor types
//                  Improved wrongLimitProtection logic
//                  Added axis user data to motor extras
//                  pollServices stop changed to emergency (limit stop) stop
// 23/03/16 M.Davis
//                  Further fixes to extractEthAddr
//                  Fixed further issues with readDataRecord header on some models
// 30/03/16 M.Clift
//                  Rewrote readDataRecord again to decrease CPU load
//                  Optimized poller cycle to reduce CPU load
//                  Optimized profile buffering to reduce CPU load
//                  Removed some unnecessary asynParams
// 01/04/16 M.Clift
//                  Fixed runProfile time base issue in sleep code
//                  Fixed runProfile motors at start test issue
//                  CSAxis in sync start only mode stop on limit changed to emergency stop type
// 19/04/16 M.Clift
//                  Further poller optimizations
//                  Changed communications from asynOctetSyncIO to asynOctet for reduced CPU load
//                  Updates to readDataRecord for tcp forced synchronous and serial modes
//                  Communications updates due to asynOctet use
//                  Fixed poller sleep time issue in synchronous mode
//                  Updated communication connection code
//                  Minor changes to wrong limit protection
//                  Updated SSI messages
//                  Fix for PWM motor selection readback
// 05/05/16 M.Clift
//                  Additional comments added
//                  Put unsolicited mesg settings back to factory default at exit
// 05/05/16 M.Davis & M.Clift
//                  Minor change to connectCallback
//                  Minor change to programUpload, and readDataRecord
// 05/05/16 M.Davis & M.Clift
//                  Stricter synchronous communication test added to GalilConnector
// 19/06/16 M.Clift
//                  Removed unused parameter GalilStopEvent
//                  Added main screen for RIO controllers
// 27/06/16 M.Clift
//                  Stop on home switch during homing now uses limit deceleration
//                  Stop on motor inhibit now uses limit deceleration
//                  Fixed axis speed change on stop
//                  Removed display code parameter from GalilStartController
// 29/06/16 M.Clift
//                  Fixed issue in programUpload method
//                  Limit update period to 200ms maximum
// 05/07/16 M.Clift
//                  Fixed time base issue in multi axis PVT profile move
// 09/07/16 M.Clift
//                  Fix communication issue when using BA cmd with external amplifier
// 18/07/16 M.Clift
//                  Fix encoded stepper motor velocity calculation in coordinated moves
//                  Add encoded stepper auxillary register synchronization with main encoder after Auto amp off delay time
//                  Fix encoded stepper motors to profile start
//                  Fix issue with acceleration transform
// 01/08/16 M.Clift
//                  Add use switch when homing parameter to motor extras
//                  Some stages don't have limit switches. eg. rotary
//                  Simplify axis home program produced by code generator
//                  SSI encoder connect detection now checks DF setting
//                  Add set homed status true on axis with a configured and connected SSI encoder
//                  Add CSAxis homed status
// 04/08/16 M.Clift
//                  Fixed move issue introduced in last commit
// 07/08/16 M.Clift
//                  Adjust update period to at least controller minimum
// 19/08/16 M.Clift
//                  Fix PID parameters autosave restoration
// 25/08/16 M.Clift
//                  Fix autosave position restore issue
//                  Add PWM servo to output compare, setPosition
//                  Add user array upload facility components
// 26/08/16 M.Clift
//                  Modification to how motorUpdateStatus request is handled
//                  Changes to QEGUI screens
//                  Changes to updatePeriod parsing
//                  Custom code can now have windows or linux line ending
//                  Thread mask can now be used to specify no thread checking
//                  Fix autosave position restore when using synchronous poller
// 29/08/16 M.Clift
//                  Add user array upload implementation completed
//                  Reduced memory requirements of profile moves
// 30/08/16 M.Clift
//                  Altered encoded open loop stepper synchronization is now performed immediately at motor stop
// 31/08/16 M.Clift
//                  Add encoded open loop stepper synchronized to encoder if stopped and encoder drifts more than retry deadband
// 11/10/16 M.Clift
//                  Altered motor amplifiers are now only disabled at driver connect time if Amp auto on/off is set to on
// 26/10/16 M.Clift
//                  Fixed maxAcceleration statistic calculated for profile motion details screen
// 26/02/17 M.Clift
//                  Fixed issue with user variable periodic monitor
//                  Fixed output compare parameter parsing
//                  Add commutation initialized flag
//                  Fixed issue with wrong limit protection becoming inactive
//                  Fixed issue with galil code variables not initialized at re-connect
//                  Add I/O Intr support for user defined variables
// 28/02/17 M.Clift
//                  Fixed galil code home variables not set after controller re-connect
// 09/03/17 M. Pearson
//                  Provide ability to set TCP port number in controller address. If no port number specified then default is 23
// 15/03/17 M.Clift
//                  Changed default encoder stall time to .2 seconds
//                  Add stop delay for heavy loads
//                  Add controller start status PV
// 25/03/17 M.Clift
//                  Fix motor velocity readback when using custom time base
//                  Add galil amplifier controls
// 31/03/17 M.Clift
//                  Optimized generated code for limits handling and homing
//                  Fixed motor enable/disable interlock variables not being set correctly
// 05/04/17 M.Clift
//                  Fixed issue with motorUpdateStatus_ when using motor record set field
//                  Fixed issue with CSAxis jog function in sync start and stop mode
//                  Improved CSAxis limit handling
// 17/04/17 M.Clift
//                  Fixed motor wont move initially after IOC start when part of CSAxis
// 22/06/17 M.Clift
//                  Removed kinematic variables
//                  Removed ability to move related CSAxis at same time
//                  Add CSAxis moves now support retries, backlash on individual motors aswell as on the CSAxis
//                  Add motors to profile start move now supports backlash, retries
//                  Add Jog after home move now supports backlash, retries
//                  Add setpoint behaviour to CSAxis
//                  Add HOMING_MONITOR PV for all axis
//                  Fixed issue with motorStatusAtHome set 1 whilst motor on limit and limit_as_home is set 0
//                  Add SSI encoder direction invert for open loop motors, and binary encoders
//                  Fixed issue with home switch homing
//                  Fixed issue programing home value after homing completion
//                  Changed home value now specified in dial coordinates
// 13/07/17 M.Clift
//                  Add enforce CSAxis completion order rules
//                  Add ability to move related CSAxis at same time when deferred moves true
//                  Add unknown unsolicited messages now routed to controller message PV for display
//                  Altered axis homeval now always 0 in dial coordinates
//                  Removed home value from motor extras
// 16/07/17 M.Clift
//                  Fix runProfile was exiting before motor completion
//                  Add motors to profile start move now has synchronous start
//                  Add update CSAxis setpoint position after homing
// 17/07/17 M.Clift
//                  Improve jog after home robustness
//                  Add validate real axis motor record settings before CSAxis home

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#if defined _WIN32 || _WIN64
#include <process.h>
#else
#include <unistd.h>
#endif /* _WIN32 */
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <typeinfo>  //std::bad_typeid
#include <sstream> //format source keys, string stream
#include <iomanip> //format source keys
#include <algorithm> //std::remove_if

using namespace std; //cout ostringstream vector string

#include <epicsString.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <errlog.h>
#include <initHooks.h>
#include <drvAsynIPPort.h>
#include <drvAsynSerialPort.h>
#include <osiSock.h>

#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <asynOctet.h>
#include <asynShellCommands.h>

#include "GalilController.h"
#include <epicsExport.h>

static const char *driverName = "GalilController";

static void GalilProfileThreadC(void *pPvt);
static void GalilArrayUploadThreadC(void *pPvt);

//Block read functions during Iocinit
//Prevent normal behaviour of output records getting/reading initial value at iocInit from driver
//Instead output records will write their db default or autosave value just after iocInit
//This change in behaviour is anticipated by the asyn record device layer and causes no error mesgs
static bool dbInitialized = false;

//Static count of Musst controllers.  Used to derive communications port name L(controller num)
static int controller_num = 0;

//Convenience functions
#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

//EPICS exit handler
static void shutdownCallback(void *pPvt)
{
  GalilController *pC_ = (GalilController *)pPvt;
  delete pC_;
}

//EPICS iocInit status
extern "C" void myHookFunction(initHookState state)
{
  //Update dbInitialized status for all GalilController instances
  if (state >= initHookAfterInitDatabase)
	dbInitialized = true;
}

//Connection status
void connectCallback(asynUser *pasynUser, asynException exception)
{
   GalilController* pC_ = (GalilController*)pasynUser->userData;
   char mesg[MAX_GALIL_STRING_SIZE];
   int connected;

   pC_->lock();
   //Update connected status
   if (exception == asynExceptionConnect)
       {
       pasynManager->isConnected(pasynUser, &connected);
       //Check that device will actually respond when asyn connected = 1
       if (connected)
          epicsEventSignal(pC_->connectEvent_);  //GalilConnector will now check for response to query
       else
          {
          //Inform user of disconnect only if GalilController connected_ is true
          if (pC_->connected_)
             {
             pC_->setIntegerParam(pC_->GalilCommunicationError_, 1);//Update connection status pv
             //If asyn connected = 0 device wont respond so go ahead and set GalilController connected_ false
             pC_->connected_ = false;
             //Give disconnect message
             sprintf(mesg, "Disconnected from %s at %s", pC_->model_, pC_->address_);
             pC_->setCtrlError(mesg);
             }
          }
       }
   pC_->unlock();
}

/** Creates a new GalilController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] address      	 The name or address to provide to Galil communication library 
  * \param[in] updatePeriod  	 The time between polls when any axis is moving
                                 If (updatePeriod < 0), polled/synchronous at abs(updatePeriod) is done regardless of bus type 
  */
GalilController::GalilController(const char *portName, const char *address, double updatePeriod)
  :  asynMotorController(portName, (int)(MAX_ADDRESS), (int)NUM_GALIL_PARAMS,
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask | asynDrvUserMask), 
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask),
                         (int)(ASYN_CANBLOCK | ASYN_MULTIDEVICE),
                         (int)1, // autoconnect
                         (int)0, (int)0),  // Default priority and stack size
  numAxes_(0), unsolicitedQueue_(MAX_GALIL_AXES, MAX_GALIL_STRING_SIZE)
{
  struct Galilmotor_enables *motor_enables = NULL;	//Convenience pointer to GalilController motor_enables[digport]
  char mesg[MAX_GALIL_STRING_SIZE];			//Connected mesg
  unsigned i;

  // Create controller-specific parameters
  createParam(GalilAddressString, asynParamOctet, &GalilAddress_);
  createParam(GalilModelString, asynParamOctet, &GalilModel_);
  createParam(GalilHomeTypeString, asynParamInt32, &GalilHomeType_);
  createParam(GalilLimitTypeString, asynParamInt32, &GalilLimitType_);
  createParam(GalilCtrlErrorString, asynParamOctet, &GalilCtrlError_);
  createParam(GalilDeferredModeString, asynParamInt32, &GalilDeferredMode_);
  createParam(GalilPVTCapableString, asynParamInt32, &GalilPVTCapable_);
  createParam(GalilStartString, asynParamInt32, &GalilStart_);

  createParam(GalilCoordSysString, asynParamInt32, &GalilCoordSys_);
  createParam(GalilCoordSysMotorsString, asynParamOctet, &GalilCoordSysMotors_);
  createParam(GalilCoordSysMovingString, asynParamInt32, &GalilCoordSysMoving_);
  createParam(GalilCoordSysSegmentsString, asynParamInt32, &GalilCoordSysSegments_);
  createParam(GalilCoordSysMotorsStopString, asynParamInt32, &GalilCoordSysMotorsStop_);
  createParam(GalilCoordSysMotorsGoString, asynParamInt32, &GalilCoordSysMotorsGo_);

  createParam(GalilProfileFileString, asynParamOctet, &GalilProfileFile_);
  createParam(GalilProfileMaxVelocityString, asynParamFloat64, &GalilProfileMaxVelocity_);
  createParam(GalilProfileMaxAccelerationString, asynParamFloat64, &GalilProfileMaxAcceleration_);
  createParam(GalilProfileMinPositionString, asynParamFloat64, &GalilProfileMinPosition_);
  createParam(GalilProfileMaxPositionString, asynParamFloat64, &GalilProfileMaxPosition_);
  createParam(GalilProfileMoveModeString, asynParamInt32, &GalilProfileMoveMode_);
  createParam(GalilProfileTypeString, asynParamInt32, &GalilProfileType_);
  createParam(GalilProfileCalculatedString, asynParamFloat64Array, &GalilProfileCalculated_);

  createParam(GalilUserArrayUploadString, asynParamInt32, &GalilUserArrayUpload_);
  createParam(GalilUserArrayString, asynParamFloat64Array, &GalilUserArray_);
  createParam(GalilUserArrayNameString, asynParamOctet, &GalilUserArrayName_);

  createParam(GalilOutputCompare1AxisString, asynParamInt32, &GalilOutputCompareAxis_);
  createParam(GalilOutputCompare1StartString, asynParamFloat64, &GalilOutputCompareStart_);
  createParam(GalilOutputCompare1IncrString, asynParamFloat64, &GalilOutputCompareIncr_);
  createParam(GalilOutputCompareMessageString, asynParamOctet, &GalilOutputCompareMessage_);

  createParam(GalilCSMotorForwardString, asynParamOctet, &GalilCSMotorForward_);
  createParam(GalilCSMotorReverseAString, asynParamOctet, &GalilCSMotorReverseA_);
  createParam(GalilCSMotorReverseBString, asynParamOctet, &GalilCSMotorReverseB_);
  createParam(GalilCSMotorReverseCString, asynParamOctet, &GalilCSMotorReverseC_);
  createParam(GalilCSMotorReverseDString, asynParamOctet, &GalilCSMotorReverseD_);
  createParam(GalilCSMotorReverseEString, asynParamOctet, &GalilCSMotorReverseE_);
  createParam(GalilCSMotorReverseFString, asynParamOctet, &GalilCSMotorReverseF_);
  createParam(GalilCSMotorReverseGString, asynParamOctet, &GalilCSMotorReverseG_);
  createParam(GalilCSMotorReverseHString, asynParamOctet, &GalilCSMotorReverseH_);

  createParam(GalilMotorSetValString, asynParamFloat64, &GalilMotorSetVal_);
  createParam(GalilMotorSetValEnableString, asynParamInt32, &GalilMotorSetValEnable_);
  createParam(GalilMotorSetString, asynParamInt32, &GalilMotorSet_);
  createParam(GalilMotorStopGoString, asynParamInt32, &GalilMotorStopGo_);

  createParam(GalilSSIConnectedString, asynParamInt32, &GalilSSIConnected_);	
  createParam(GalilEncoderStallTimeString, asynParamFloat64, &GalilEStallTime_);

  createParam(GalilStepSmoothString, asynParamFloat64, &GalilStepSmooth_);
  createParam(GalilMotorTypeString, asynParamInt32, &GalilMotorType_);

  createParam(GalilMotorConnectedString, asynParamInt32, &GalilMotorConnected_);

  createParam(GalilAfterLimitString, asynParamFloat64, &GalilAfterLimit_);
  createParam(GalilWrongLimitProtectionString, asynParamInt32, &GalilWrongLimitProtection_);
  createParam(GalilWrongLimitProtectionActiveString, asynParamInt32, &GalilWrongLimitProtectionActive_);

  createParam(GalilUserOffsetString, asynParamFloat64, &GalilUserOffset_);
  createParam(GalilEncoderResolutionString, asynParamFloat64, &GalilEncoderResolution_);
  createParam(GalilUseEncoderString, asynParamInt32, &GalilUseEncoder_);
  createParam(GalilStopPauseMoveGoString, asynParamInt32, &GalilStopPauseMoveGo_);

  createParam(GalilPremString, asynParamOctet, &GalilPrem_);
  createParam(GalilPostString, asynParamOctet, &GalilPost_);

  createParam(GalilUseSwitchString, asynParamInt32, &GalilUseSwitch_);
  createParam(GalilUseIndexString, asynParamInt32, &GalilUseIndex_);
  createParam(GalilJogAfterHomeString, asynParamInt32, &GalilJogAfterHome_);
  createParam(GalilJogAfterHomeValueString, asynParamFloat64, &GalilJogAfterHomeValue_);

  createParam(GalilAutoOnOffString, asynParamInt32, &GalilAutoOnOff_);
  createParam(GalilAutoOnDelayString, asynParamFloat64, &GalilAutoOnDelay_);
  createParam(GalilAutoOffDelayString, asynParamFloat64, &GalilAutoOffDelay_);

  createParam(GalilAutoBrakeString, asynParamInt32, &GalilAutoBrake_);
  createParam(GalilAutoBrakeOnDelayString, asynParamFloat64, &GalilAutoBrakeOnDelay_);
  createParam(GalilBrakePortString, asynParamInt32, &GalilBrakePort_);
  createParam(GalilBrakeString, asynParamInt32, &GalilBrake_);
  createParam(GalilHomeAllowedString, asynParamInt32, &GalilHomeAllowed_);
  createParam(GalilStopDelayString, asynParamFloat64, &GalilStopDelay_);

  createParam(GalilAmpGainString, asynParamInt32, &GalilAmpGain_);
  createParam(GalilAmpCurrentLoopGainString, asynParamInt32, &GalilAmpCurrentLoopGain_);
  createParam(GalilAmpLowCurrentString, asynParamInt32, &GalilAmpLowCurrent_);
  createParam(GalilHomingString, asynParamInt32, &GalilHoming_);
  createParam(GalilUserDataString, asynParamFloat64, &GalilUserData_);
  createParam(GalilUserDataDeadbString, asynParamFloat64, &GalilUserDataDeadb_);

  createParam(GalilMainEncoderString, asynParamInt32, &GalilMainEncoder_);
  createParam(GalilAuxEncoderString, asynParamInt32, &GalilAuxEncoder_);
  createParam(GalilMotorAcclString, asynParamFloat64, &GalilMotorAccl_);
  createParam(GalilMotorRdbdString, asynParamFloat64, &GalilMotorRdbd_);
  createParam(GalilMotorVeloString, asynParamFloat64, &GalilMotorVelo_);
  createParam(GalilMotorVmaxString, asynParamFloat64, &GalilMotorVmax_);

  createParam(GalilBinaryInString, asynParamUInt32Digital, &GalilBinaryIn_);
  createParam(GalilBinaryOutString, asynParamUInt32Digital, &GalilBinaryOut_);
  createParam(GalilBinaryOutRBVString, asynParamUInt32Digital, &GalilBinaryOutRBV_);

  createParam(GalilAnalogInString, asynParamFloat64, &GalilAnalogIn_);
  createParam(GalilAnalogInDeadbString, asynParamFloat64, &GalilAnalogInDeadb_);

  createParam(GalilAnalogOutString, asynParamFloat64, &GalilAnalogOut_);
  createParam(GalilAnalogOutRBVString, asynParamFloat64, &GalilAnalogOutRBV_);
  createParam(GalilAnalogOutRBVDeadbString, asynParamFloat64, &GalilAnalogOutRBVDeadb_);

  createParam(GalilDirectionString, asynParamInt32, &GalilDirection_);
  createParam(GalilDmovString, asynParamInt32, &GalilDmov_);
  createParam(GalilSSICapableString, asynParamInt32, &GalilSSICapable_);

  createParam(GalilSSIInputString, asynParamInt32, &GalilSSIInput_);
  createParam(GalilSSITotalBitsString, asynParamInt32, &GalilSSITotalBits_);
  createParam(GalilSSISingleTurnBitsString, asynParamInt32, &GalilSSISingleTurnBits_);
  createParam(GalilSSIErrorBitsString, asynParamInt32, &GalilSSIErrorBits_);
  createParam(GalilSSITimeString, asynParamInt32, &GalilSSITime_);
  createParam(GalilSSIDataString, asynParamInt32, &GalilSSIData_);
  createParam(GalilSSIInvertString, asynParamInt32, &GalilSSIInvert_);
  createParam(GalilErrorLimitString, asynParamFloat64, &GalilErrorLimit_);
  createParam(GalilErrorString, asynParamFloat64, &GalilError_);
  createParam(GalilOffOnErrorString, asynParamInt32, &GalilOffOnError_);
  createParam(GalilAxisString, asynParamInt32, &GalilAxis_);
  createParam(GalilMotorVelocityEGUString, asynParamFloat64, &GalilMotorVelocityEGU_);
  createParam(GalilMotorVelocityRAWString, asynParamFloat64, &GalilMotorVelocityRAW_);

  createParam(GalilUserCmdString, asynParamFloat64, &GalilUserCmd_);
  createParam(GalilUserOctetString, asynParamOctet, &GalilUserOctet_);
  createParam(GalilUserOctetValString, asynParamFloat64, &GalilUserOctetVal_);
  createParam(GalilUserVarString, asynParamFloat64, &GalilUserVar_);

  createParam(GalilEthAddrString, asynParamOctet, &GalilEthAddr_);
  createParam(GalilSerialNumString, asynParamOctet, &GalilSerialNum_);

//Add new parameters here

  createParam(GalilCommunicationErrorString, asynParamInt32, &GalilCommunicationError_);

  //Store address
  strcpy(address_, address);
  //Default model
  strcpy(model_, "Unknown");
  //Code for the controller has not been assembled yet
  code_assembled_ = false;
  //We have not recieved a timeout yet
  consecutive_timeouts_ = 0;
  //Assume sync tcp mode will be used for now
  async_records_ = false;
  //Determine if we should even try async udp before going to synchronous tcp mode 
  try_async_ = (updatePeriod < 0) ? false : true;
  //Store absolute period in ms between data records
  updatePeriod_ = fabs(updatePeriod);
  //Limit maximum update period
  if (updatePeriod_ > MAX_UPDATE_PERIOD)
     {
     sprintf(mesg, "Capping UpdatePeriod to %dms maximum", MAX_UPDATE_PERIOD);
     setCtrlError(mesg);
     updatePeriod_ = MAX_UPDATE_PERIOD;
     }
  //Code generator has not been initialized
  codegen_init_ = false;	
  digitalinput_init_ = false;
  //Motor enables defaults
  digports_ = 0;
  digvalues_ = 0;
  //Deferred moves off at start-up
  movesDeferred_ = false;
  //Store the controller number for later use
  controller_number_ = controller_num;
  //Allocate memory for code buffers.  
  //We put all code for this controller in these buffers.
  thread_code_ = (char *)calloc(MAX_GALIL_AXES * (THREAD_CODE_LEN),sizeof(char));	
  limit_code_ = (char *)calloc(MAX_GALIL_AXES * (LIMIT_CODE_LEN),sizeof(char));
  digital_code_ = (char *)calloc(MAX_GALIL_AXES * (INP_CODE_LEN),sizeof(char));
  card_code_ = (char *)calloc(MAX_GALIL_AXES * (THREAD_CODE_LEN+LIMIT_CODE_LEN+INP_CODE_LEN),sizeof(char));
  user_code_ = (char *)calloc(MAX_GALIL_AXES * (THREAD_CODE_LEN+LIMIT_CODE_LEN+INP_CODE_LEN),sizeof(char));
  //zero code buffers
  strcpy(thread_code_, "");
  strcpy(limit_code_, "");
  strcpy(digital_code_, "");
  strcpy(card_code_, "");
 
  //Set defaults in Paramlist before connect
  setParamDefaults();

  //Register for iocInit state updates, so we can keep track of iocInit status
  initHookRegister(myHookFunction);

  //Create the event that wakes up the GalilConnector thread
  connectEvent_ = epicsEventMustCreate(epicsEventEmpty);

  //Create connector thread that manages connection status flags
  connector_ = new GalilConnector(this);

  // Create the event that wakes up the thread for profile moves
  profileExecuteEvent_ = epicsEventMustCreate(epicsEventEmpty);

  // Create the event that wakes up the thread for array upload
  arrayUploadEvent_ = epicsEventMustCreate(epicsEventEmpty);
  
  // Create the thread that will execute profile moves
  epicsThreadCreate("GalilProfile",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)GalilProfileThreadC, (void *)this);

  // Create the thread that will upload arrays
  epicsThreadCreate("GalilArrayUpload",
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)GalilArrayUploadThreadC, (void *)this);
                    
  //Initialize the motor enables struct in GalilController instance
  for (i=0;i<8;i++)
	{
	//Retrieve structure for digital port from controller instance
	motor_enables = (Galilmotor_enables *)&motor_enables_[i];
	//Initialize motor enables structure in GalilController instance
	strcpy(motor_enables->motors, "");
	strcpy(motor_enables->disablestates, "");
	}

  //Thread to acquire datarecord for a single GalilController
  //We write our own because communications with controller is rather unique
  poller_ = new GalilPoller(this);

  //Put, and wait until poller is in sleep mode
  //Also stop async records from controller
  poller_->sleepPoller();

  //Establish the initial connection to controller
  connect();

  //Static count of controllers.  Used to derive communications port names
  controller_num++;
}

//Called by GalilController at start up once only.  
//Asyn does connection management for us, and gives callbacks at status change
void GalilController::connect(void)
{
  asynInterface *pasynInterface;		//To retrieve required asyn interfaces
  char address_string[MAX_GALIL_STRING_SIZE];	//Temporary address string used to setup communications
  int sync_connected;			//Is the synchronous communication socket connected according to asyn
  int async_connected = 1;		//Is the asynchronous communication connected according to asyn
  std::string address = address_;	//Convert address into std::string for easy inspection

  //Set default timeout at connect
  timeout_ = 1;

  //Construct the asyn port name that will be used for synchronous communication
  sprintf(syncPort_, "GALILSYNC%d", controller_number_);
  if (address.find("COM") == string::npos && address.find("ttyS") == string::npos)
     {
     //Open Synchronous ethernet connection
     //If no port number is specified in the provided address, append Telnet port and TCP directive
     //Otherwise just append the TCP directive
     if (strchr(address_, ':') == NULL) {
        sprintf(address_string,"%s:23 TCP", address_);
     } else {
        sprintf(address_string,"%s TCP", address_);
     }
     //Connect to the device, we don't want end of string processing
     drvAsynIPPortConfigure(syncPort_, address_string, epicsThreadPriorityMedium, 0, 1);

     if (try_async_)
        {
        //Create Asynchronous udp connection
        //Construct the asyn port name that will be used for asynchronous UDP communication
        sprintf(asyncPort_, "GALILASYNC%d", controller_number_);
        //Construct address to use for udp server
        sprintf(address_string,"%s:60007 udp", address_);
        //Connect to the device, we don't want end of string processing
        drvAsynIPPortConfigure(asyncPort_, address_string, epicsThreadPriorityMax, 0, 1);
        //Connect to asyn communications port created above and return pasynUser for async communication
        pasynOctetSyncIO->connect(asyncPort_, 0, &pasynUserAsyncGalil_, NULL);
        //Retrieve async octet interface for reading the data record
        pasynInterface = pasynManager->findInterface(pasynUserAsyncGalil_, asynOctetType, 1);
        //Store the asyn octet interface for async connection in GalilController
        pAsyncOctet_ = (asynOctet *)pasynInterface->pinterface;
        pAsyncOctetPvt_ = pasynInterface->drvPvt;
        //Retrieve asynchronous connection status from asyn
        pasynManager->isConnected(pasynUserAsyncGalil_, &async_connected);
        //For tracing/debugging async communications
        //asynSetTraceMask(asyncPort_,-1,0xFF);
        //asynSetTraceIOMask(asyncPort_,-1,0xFF);
        }
     }
  else
     {
     //Connect to the device, we don't want end of string processing
     drvAsynSerialPortConfigure(syncPort_, address_, epicsThreadPriorityMax, 0, 1);
     //Flag try_async_ records false for serial connections
     try_async_ = false;
     }

  //Connect to synchronous communications port created above and return pasynUser for sync communication
  pasynOctetSyncIO->connect(syncPort_, 0, &pasynUserSyncGalil_, NULL);
  //Store GalilController instance in asynUser for later access
  pasynUserSyncGalil_->userData = this;
  //Add asyn exception callback to capture asyn connected change of state
  pasynManager->exceptionCallbackAdd(pasynUserSyncGalil_, connectCallback);
  //Retrieve asyn common interface for forced disconnect on synchronous communications
  pasynInterface = pasynManager->findInterface(pasynUserSyncGalil_, asynCommonType, 1);
  //Store the asyn common interface for sync in GalilController instance for use during forced disconnect
  pasynCommon_ = (asynCommon *)pasynInterface->pinterface;
  pcommonPvt_ = pasynInterface->drvPvt;
  //Retrieve sync octet interface for reading the data record
  pasynInterface = pasynManager->findInterface(pasynUserSyncGalil_, asynOctetType, 1);
  //Store the asyn octet interface for sync connection in GalilController
  pSyncOctet_ = (asynOctet *)pasynInterface->pinterface;
  pSyncOctetPvt_ = pasynInterface->drvPvt;
  //Retrieve synchronous connection status from asyn
  pasynManager->isConnected(pasynUserSyncGalil_, &sync_connected);
  //For tracing/debugging sync communications
  //asynSetTraceMask(syncPort_,-1,0xFF);
  //asynSetTraceIOMask(syncPort_,-1,0xFF);

  //Set an EPICS exit handler that will shut down polling before exit
  //Must be done after drvAsyn port configure so our exit handler is called before asyn one at epics exit
  epicsAtExit(shutdownCallback, this);

  //GalilConnector will now check for response to query and set connection status flag
  if (sync_connected && async_connected)
     epicsEventSignal(connectEvent_);

  //Pause long enough for GalilConnector to update connection status flags, and show connection message
  epicsThreadSleep(.5);
}

//Called when async poll fails
//Forces asyn to disconnect/reconnect
void GalilController::disconnect(void)
{
   int connected;

   //Retrieve asyn connected status
   pasynManager->isConnected(pasynUserSyncGalil_, &connected);
   //Disconnect asyn from device if asyn connected = 1
   if (connected)
      {
      //Inform asyn we are going to disconnect
      pasynManager->exceptionDisconnect(pasynUserSyncGalil_);
      //Disconnect from the controller.  This will cause connectCallback
      pasynCommon_->disconnect(pcommonPvt_, pasynUserSyncGalil_);
      }
}

void GalilController::shutdownController()
{
   unsigned i;
   GalilAxis *pAxis;
   GalilCSAxis *pCSAxis;

   //Destroy the poller for this GalilController.
   if (poller_ != NULL)
      {
      delete poller_;
      poller_ = NULL;
      }

   //Destroy the connector thread for this GalilController
   if (connector_ != NULL)
      {
      delete connector_; // this will close all connections and delete GalilController etc 
      connector_ = NULL;
      }

   //Free the memory where card code is stored
   free(card_code_);

   //Free any GalilAxis, and GalilCSAxis instances
   for (i = 0; i < MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
      {
      if (i < MAX_GALIL_AXES)
         {
         pAxis = getAxis(i);
         if (pAxis)
            delete pAxis;
         }
      else
        {
        pCSAxis = getCSAxis(i);
        if (pCSAxis) 
           delete pCSAxis;
        }
      }

   //Burn parameters, and cleanup
   if (connected_)
      {
      //Obtain the lock
      lock();
      //Burn parameters on exit ensuring controller has correct settings at next power on
      //This effects motor type, soft limits, limit configuration etc
      //It does not effect the galil program on the controller
      sprintf(cmd_, "BN");
      sync_writeReadController();
      //Configure serial for unsolicited at exit as per factory default
      strcpy(cmd_, "CF S");
      sync_writeReadController();
      //Configure to not set MSB for unsolicited messages at exit as per factory default
      strcpy(cmd_, "CW 2");
      sync_writeReadController();
      if (async_records_)
         {
         //Close all udp async connections
         strcpy(cmd_, "IHT=>-1");
         sync_writeReadController();
         }
      //Release the lock
      unlock();
      //Asyn exit handler will disconnect sync connection from here
      //We just print message to tell user Asyn epicsAtExit callback is running (next) and will close connection
      cout << "Disconnecting from " << model_ << " at " << address_ << endl;
      }
}

GalilController::~GalilController()
{
	shutdownController();
}

void GalilController::setParamDefaults(void)
{ 
  unsigned i;
  //Set defaults in Paramlist before connected
  //Pass address string provided by GalilCreateController to upper layers
  setStringParam(GalilAddress_, address_);
  //Set default model string
  setStringParam(GalilModel_, "Unknown");
  //SSI capable
  setIntegerParam(GalilSSICapable_, 0);
  //PVT capable
  setIntegerParam(GalilPVTCapable_, 0);
  //Communication status
  setIntegerParam(GalilCommunicationError_, 1);

  //Deferred moves off 
  setIntegerParam(motorDeferMoves_, 0);
  //Default coordinate system is S
  setIntegerParam(GalilCoordSys_, 0);
  //Coordinate system S axes list empty
  setStringParam(0, GalilCoordSysMotors_, "");
  //Coordinate system T axes list empty
  setStringParam(1, GalilCoordSysMotors_, "");
  //Put all motors in spmg go mode
  for (i = 0; i < MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
	setIntegerParam(i, GalilMotorStopGo_, 3);
  //Output compare is off
  for (i = 0; i < 2; i++)
	setIntegerParam(i, GalilOutputCompareAxis_, 0);
  setStringParam(GalilSerialNum_, "");
  setStringParam(GalilEthAddr_, "");
  //Default all forward kinematics to null strings
  for (i = MAX_GALIL_CSAXES; i < MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
     setStringParam(i, GalilCSMotorForward_, "");
  //Default controller error message to null string
  setStringParam(0, GalilCtrlError_, "");
}

// extract the controller ethernet address from the output of the galil TH command
// return 00-00-00-00-00-00 if unable to parse input string
std::string GalilController::extractEthAddr(const char* str)
{
	//Result string
	std::string mac;
	//Search string
	static const std::string eth("ETHERNET ADDRESS");
	//Result from TH command
	std::string th(str);
	//Find start and end of substring containing mac address
	size_t start = th.find(eth);
	start = start + eth.size() + 1;
	size_t end = start + 17;
	//copy portion of the string containing the MAC addr
	if ((start < 0) || (end >= th.size()) || (start >= end))
		mac = std::string("00-00-00-00-00-00");  //tdebug
	else
		mac = std::string(th.substr(start, end - start));
	//Trim final result to mac address length
	mac.resize(17);
	return mac;
}

//Anything that should be done once connection established
//Read controller details, stop all motors and threads
void GalilController::connected(void)
{
  //static const char *functionName = "connected";
  char RV[] = {0x12,0x16,0x0};  //Galil command string for model and firmware version query
  int status;
  double minUpdatePeriod;		//Min update period given model
  char mesg[MAX_GALIL_STRING_SIZE];	//Connected mesg
  unsigned i;

  //Flag connected as true
  connected_ = true;
  setIntegerParam(GalilCommunicationError_, 0);
  //Load model, and firmware query into cmd structure
  strcpy(cmd_, RV);
  //Query model, and firmware version
  sync_writeReadController();
  //store model, and firmware version in GalilController instance
  strcpy(model_, resp_);
  //Pass model string to ParamList
  setStringParam(GalilModel_, model_);
  //Determine if controller is dmc or rio
  rio_ = (strncmp(model_, "RIO",3) == 0) ? true : false;

  //Give connect message
  sprintf(mesg, "Connected to %s at %s", model_, address_);
  setCtrlError(mesg);

  //Determine max number of axes the controller supports
  //Note Galil BA command will reduce axis on a controller, and alter
  //the returned model revision string
  if (!rio_)
     {
     if (model_[3] == '5') //DMC50000 series
        numAxesMax_ = model_[6] - ZEROASCII;
     else if (model_[3] == '3') //DMC30000 series
        numAxesMax_ = 1;
     else //DMC21x3, DMC41x3, DMC40x0
        numAxesMax_ = model_[5] - ZEROASCII;
     }
  else //RIO PLC
     numAxesMax_ = 0;

  //Parse provided updatePeriod given model minimum
  if ((model_[3] == '2' || model_[3] == '4') && model_[4] == '1' && !rio_)
     minUpdatePeriod = 8; //Econo series controllers 8 ms min
  else
     minUpdatePeriod = 2; //All others 2 ms min
  if (updatePeriod_ < minUpdatePeriod)//Re-adjust update time to controller min if necessary
     {
     sprintf(mesg, "Restricting UpdatePeriod to %.0lfms minimum", minUpdatePeriod);
     setCtrlError(mesg);
     updatePeriod_ = minUpdatePeriod;
     }
  
  //Read Ethernet handle details
  strcpy(cmd_, "TH");
  sync_writeReadController();
  setStringParam(GalilEthAddr_, extractEthAddr(resp_).c_str());

  //Read serial number
  strcpy(cmd_, "MG _BN");
  sync_writeReadController();
  setStringParam(GalilSerialNum_, resp_);

  //Determine if controller is SSI capable
  if (strstr(model_, "SSI") != NULL || strstr(model_, "SER") != NULL)
     setIntegerParam(GalilSSICapable_, 1);
  else
     setIntegerParam(GalilSSICapable_, 0);

  //Determine if controller is PVT capable
  if (model_[3] == '5' || model_[3] == '4' || model_[3] == '3')
     setIntegerParam(GalilPVTCapable_, 1);
  else
     setIntegerParam(GalilPVTCapable_, 0);

  //Determine maximum acceleration
  //default for most models
  maxAcceleration_ = 1073740800;
  //DMC50000
  maxAcceleration_ = (model_[3] == '5') ? 2147483648 : maxAcceleration_;
  //DMC2xxx
  maxAcceleration_ = (model_[3] == '2') ? 67107840 : maxAcceleration_;
	
  //Determine number of threads supported
  //Safe default
  numThreads_ = 8;
  //Check for controllers that support < 8 threads
  //RIO
  numThreads_ = (rio_)? 4 : numThreads_;
  //DMC3 range
  if ((model_[0] == 'D' && model_[3] == '3'))
     numThreads_ = 6;
  //DMC1 range
  numThreads_ = (model_[3] == '1')? 2 : numThreads_;

  //Stop all threads running on the controller
  for (i=0;i<numThreads_;i++)
     {
     sprintf(cmd_, "HX%d",i);
     sync_writeReadController();
     }

  //Stop all moving motors
  for (i=0;i<numAxesMax_;i++)
     {
     //Query moving status
     sprintf(cmd_, "MG _BG%c", (i + AASCII));
     sync_writeReadController();
     if (atoi(resp_))
        {
        //Stop moving motor
        sprintf(cmd_, "ST%c", (i + AASCII));
        sync_writeReadController();
        //Allow time for motor stop
        epicsThreadSleep(1.0);
        //Ensure home process is stopped
        sprintf(cmd_, "home%c=0", (i + AASCII));
        sync_writeReadController();
        }
     }

  //Initialize data record structures
  InitializeDataRecord();

  //No timeout errors
  consecutive_timeouts_ = 0;

  //Has code for the GalilController been assembled
  if (code_assembled_)
     {
     //This is reconnect
     //Put poller to sleep for GalilStartController
     unlock();
     poller_->sleepPoller();
     lock();
     //Deliver and start the code on controller
     GalilStartController(code_file_, burn_program_, thread_mask_);
     }
  else
     {
     //This is initial connect
     //Try async udp mode unless user specfically wants sync tcp mode
     if (try_async_)
        {
        //Start async data record transmission on controller
        sprintf(cmd_, "DR %.0f, %d", updatePeriod_, udpHandle_ - AASCII);
        status = sync_writeReadController();
        if (status)
           {
           async_records_ = false; //Something went wrong
           setCtrlError("Asynchronous UDP failed, switching to TCP synchronous");
           }
        else
           async_records_ = true; //All ok
        }
     }

  //Set connection that will receive unsolicited messages
  if (async_records_)
     sprintf(cmd_, "CF %c", udpHandle_);
  else
     sprintf(cmd_, "CF %c", syncHandle_);
  status = sync_writeReadController();

  //Set most signficant bit for unsolicited bytes
  strcpy(cmd_, "CW 1");
  status = sync_writeReadController();

  callParamCallbacks();
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void GalilController::report(FILE *fp, int level)
{
  //int axis;
  //GalilAxis *pAxis;

  fprintf(fp, "Galil motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
  /*
  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %d\n"
              "    pulsesPerUnit_ = %f\n"
              "    encoder position=%f\n"
              "    theory position=%f\n"
              "    limits=0x%x\n"
              "    flags=0x%x\n", 
              pAxis->axisNo_, pAxis->pulsesPerUnit_, 
              pAxis->encoderPosition_, pAxis->theoryPosition_,
              pAxis->currentLimits_, pAxis->currentFlags_);
    }
  }*/

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an GalilMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
GalilAxis* GalilController::getAxis(asynUser *pasynUser)
{
  //For real motors
  return static_cast<GalilAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an GalilMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
GalilCSAxis* GalilController::getCSAxis(asynUser *pasynUser)
{
  //For coordinate system motors
  return static_cast<GalilCSAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an GalilMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
GalilAxis* GalilController::getAxis(int axisNo)
{
  //For real motors
  return static_cast<GalilAxis*>(asynMotorController::getAxis(axisNo));
}

/** Returns a pointer to an GalilMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
GalilCSAxis* GalilController::getCSAxis(int axisNo)
{
  //For coordinate system motors
  return static_cast<GalilCSAxis*>(asynMotorController::getAxis(axisNo));
}

/** Returns true if any motor in the provided list is moving
  * Motors stopped less than offdelay are still considered as moving here
  * \param[in] Motor list
  */
bool GalilController::anyMotorMoving()
{
  GalilAxis *pAxis;	//GalilAxis instance
  int moving = 0;	//Moving status
  int dmov;		//GalilAxis dmov
  int i;		//Looping

  //Look through motor list, if any moving return true
  for (i = 0; profileAxes_[i] != '\0'; i++)
	{
	//Retrieve the axis
	pAxis = getAxis(profileAxes_[i] - AASCII);
	//Process or skip
	if (!pAxis) continue;
	//Determine moving status
	getIntegerParam(pAxis->axisNo_, motorStatusMoving_, &moving);
	getIntegerParam(pAxis->axisNo_, GalilDmov_, &dmov);
	//Motor considered moving until all retries, backlash completed
	if (moving || !dmov) return true;
	}

  //None of the motors were moving
  return false;
}

/** Returns true if all motors in the provided list are moving
  * Takes retries and backlash into account
  * \param[in] Motor list
  */
bool GalilController::allMotorsMoving(char *axes)
{
  int moving;		//Moving status
  GalilAxis *pAxis;	//GalilAxis instance
  int dmov;		//GalilAxis dmov
  int i;		//Looping

  //Look through motor list, if any not moving return false
  for (i = 0; axes[i] != '\0'; i++)
	{
	//Retrieve the axis
	pAxis = getAxis(axes[i] - AASCII);
	//Process or skip
	if (!pAxis) continue;
	//Determine moving status
	getIntegerParam(pAxis->axisNo_, motorStatusMoving_, &moving);
	getIntegerParam(pAxis->axisNo_, GalilDmov_, &dmov);
	//Motor considered stopped if not moving, and all retries, backlash completed
	if (!moving && dmov) return false;
	}

  //All the motors were moving
  return true;
}

/** Returns true if motors are moving or stopped at start position without limit
  * Returns false a single motor has stopped out of position, or with limit
  * \param[in] - Requested start positions Units=Steps
  */
bool GalilController::motorsAtStart(double startp[])
{
  char message[MAX_GALIL_STRING_SIZE];	//Profile execute message
  GalilAxis *pAxis;	//Real motor instance
  bool atStart = true;	//Are all motors in axes list moving or stopped without limit
  int moveMode;		//Move mode absolute or relative
  int moving;		//Axis moving status
  int dmov;		//Axis done moving
  int fwd, rev;		//Axis fwd, rev limit status
  int axisNo;		//Axis number
  double readback;	//Axis position readback in egu
  double start;		//Desired motor start position in egu
  double eres, mres;	//Encoder, and motor resolution
  double epos, mpos;	//Encoder, motor position
  double off;		//User offset
  int dir, dirm;	//Direction multiplier
  int ueip;		//Use encoder if present
  double rdbd;		//Motor record retry deadband
  int i;		//Looping

  //Look through motor list
  for (i = 0; profileAxes_[i] != '\0'; i++)
     {
     //Determine axis number
     axisNo = profileAxes_[i] - AASCII;
     //Retrieve the axis
     pAxis = getAxis(axisNo);
	 //Process or skip
     if (!pAxis) continue;
     //Retrieve GalilProfileMoveMode_ from ParamList
     getIntegerParam(axisNo, GalilProfileMoveMode_, &moveMode);
     //If moveMode = Relative skip the axis
     if (!moveMode) continue;
     //Retrieve needed parameters
     getIntegerParam(axisNo, GalilDmov_, &dmov);
     getIntegerParam(axisNo, motorStatusMoving_, &moving);
     getIntegerParam(axisNo, motorStatusLowLimit_, &rev);
     getIntegerParam(axisNo, motorStatusHighLimit_, &fwd);
     getDoubleParam(axisNo, motorResolution_, &mres);
     getDoubleParam(axisNo, GalilEncoderResolution_, &eres);
     getDoubleParam(axisNo, GalilMotorRdbd_, &rdbd);
     getIntegerParam(axisNo, GalilUseEncoder_, &ueip);
     getIntegerParam(axisNo, GalilDirection_, &dir);
     getDoubleParam(axisNo, GalilUserOffset_, &off);
     getDoubleParam(axisNo, motorEncoderPosition_, &epos);
     getDoubleParam(axisNo, motorPosition_, &mpos);
     //Calculate direction multiplier
     dirm = (dir == 0) ? 1 : -1;
     //Calculate readback in user coordinates
     readback = (ueip) ? (epos * eres * dirm) + off : (mpos * mres * dirm) + off;
     //Calculate the motor start position in user coordinates
     start = (startp[axisNo] * mres * dirm) + off;
     //Determine result
     if ((!moving && dmov && (readback < (start - rdbd))) || (!moving && dmov && (readback > (start + rdbd))) || (!moving && (rev || fwd)))
        {
        atStart = false;
        break;
        }
     }

  if (!atStart)
     {
     //Store message in paramList
     if (rev || fwd)
        sprintf(message, "Profile motor %c hit limit whilst moving to profile start position", axisNo + AASCII);
     else
        sprintf(message, "Profile motor %c at %2.4lf did not reach start %2.4lf within retry deadband %2.4lf", axisNo + AASCII, readback, start, rdbd);
     setStringParam(profileExecuteMessage_, message);
     }

  //Motors were all at profile start position
  return atStart;
}

/** setOutputCompare function.  For turning output compare on/off
  * \param[in] output compare to setup - 0 or 1 for output compare 1 and 2 */
asynStatus GalilController::setOutputCompare(int oc)
{
  char message[MAX_GALIL_STRING_SIZE];	//Output compare message
  int ocaxis;				//Output compare axis from paramList
  int axis;				//Axis number derived from output compare axis in paramList
  int motor;				//motor type read from controller
  int start, end;			//Looping
  double ocstart;			//Output compare start position from paramList
  double ocincr;			//Output compare incremental distance for repeat pulses from paramList
  double eres;	        		//mr eres
  int mainencoder, auxencoder;		//Main and aux encoder setting
  int encoder_setting;			//Overall encoder setting value
  bool encoders_ok = false;		//Encoder setting good or bad for output compare
  bool setup_ok = false;		//Overall setup status
  int comstatus = asynSuccess;		//Status of comms
  int paramstatus = asynSuccess;	//Status of paramList gets
  int i;				//Looping

  //Output compare not valid on some controllers
  //Output compare 1 valid for controllers > 0 axis
  //Output compare 2 valid for controllers > 4 axis
  if ((oc && numAxesMax_ <= 4) || (!oc && !numAxesMax_))
     return asynSuccess;

  //Retrieve axis to use with output compare
  paramstatus = getIntegerParam(oc, GalilOutputCompareAxis_, &ocaxis);

  //Attempt turn on selected output compare
  if (ocaxis && !paramstatus)
	{
	//Convert paramList ocaxis to 0-7 for axis A-H
	axis = (oc) ? (ocaxis - 1 + 4) : (ocaxis - 1);
	//Query motor type
	sprintf(cmd_, "MT%c=?", axis + AASCII);
	comstatus = sync_writeReadController();
	motor = atoi(resp_);
	//Check encoder settings
	paramstatus |= getIntegerParam(axis, GalilMainEncoder_, &mainencoder);
	paramstatus |= getIntegerParam(axis, GalilAuxEncoder_, &auxencoder);
	encoder_setting = mainencoder + auxencoder;
	//If main, and auxillary encoder setting match
	if ((!encoder_setting || encoder_setting == 5 || encoder_setting == 10 || encoder_setting == 15) && !paramstatus)
		encoders_ok = true;
	//If motor is a servo, and encoder settings match, no paramlist error, and no command error
	if ((abs(motor) == 1 || abs(motor) == 1.5) && encoders_ok && !paramstatus && !comstatus)
		{
		//Passed motor configuration checks.  Motor is servo, and encoder setting is ok
		//Retrieve output compare start, and increment values
		paramstatus |= getDoubleParam(oc, GalilOutputCompareStart_, &ocstart);
		paramstatus |= getDoubleParam(oc, GalilOutputCompareIncr_, &ocincr);
		//Retrieve the axis encoder resolution
		paramstatus |= getDoubleParam(axis, GalilEncoderResolution_, &eres);
		//Convert start and increment to steps
		ocstart = ocstart / eres;
		ocincr = ocincr / eres;
		//Check start, and increment values
		if (rint(ocincr) >= -65536 && rint(ocincr) <= 65535 &&
		    rint(ocstart) >= -2147483648 && rint(ocstart) <= 2147483647 && !paramstatus)
			{			
			if (!paramstatus)
				{
				sprintf(cmd_, "OC%c=%.0lf,%.0lf", axis + AASCII, rint(ocstart), rint(ocincr));
				comstatus = sync_writeReadController();
				setup_ok = (!comstatus) ? true : false;
				if (setup_ok)
					{
					sprintf(message, "Output compare %d setup successfully", oc + 1);
					setStringParam(GalilOutputCompareMessage_, message);
					}
				else
					{
					//Reject motor setting if problem
					sprintf(message, "Output compare %d setup failed", oc + 1);
					setStringParam(GalilOutputCompareMessage_, message);
					setIntegerParam(oc, GalilOutputCompareAxis_, 0);
					}
				}
			}
		else
			{
			//Reject motor setting if problem with start or increment
			sprintf(message, "Output compare %d failed due to start/increment out of range", oc + 1);
			setStringParam(GalilOutputCompareMessage_, message);
			setIntegerParam(oc, GalilOutputCompareAxis_, 0);
			}
		}
	else if (!paramstatus && !comstatus)
		{
		//Reject motor setting if the motor has a configuration problem
		sprintf(message, "Output compare %d failed due to configuration problem axis %c", oc + 1, axis + AASCII);
		paramstatus = setStringParam(GalilOutputCompareMessage_, message);
		setIntegerParam(oc, GalilOutputCompareAxis_, 0);
		}
	}

  //Attempt turn off selected output compare
  if (!setup_ok && !paramstatus)
	{
	//Default parameters
	axis = 99;
	//Calculate loop start/end
	start = (!oc) ? 0 : 4;
	end = (!oc) ? 4 : 8;
	
	//Find a servo in correct bank either A-D, or bank E-H
	for (i = start; i < end; i++)
		{
		sprintf(cmd_, "MT%c=?", i + AASCII);
		comstatus = sync_writeReadController();
		motor = atoi(resp_);
		if (abs(motor) == 1 || abs(motor) == 1.5)
			{
			axis = i;
			break;
			}
		}
	
	if (axis != 99)
		{
		//A servo was found in the correct bank
		sprintf(cmd_, "OC%c=0,0", axis + AASCII);
		comstatus = sync_writeReadController();
		if (!ocaxis)
			{
			sprintf(message, "Output compare %d turned off", oc + 1);
			setStringParam(GalilOutputCompareMessage_, message);
			}
		}
	}

  return (asynStatus)comstatus;
}

/*
 * Called at start of profile build, and execute
 * Check CSAxis profiles, make sure motors are not shared amongst specified CSAxis.
 * Check useAxis consistent amongst CSAxis and their reverse axis (real motors)
 * Check moveMode consistent amongst CSAxis and their reverse axis (real motors)
 * Param [in] mesg_function - Function number of message receiver (ie. profileBuildMessage_ or profileExecuteMessage_)
 * @return motor driver status code.
 */
asynStatus GalilController::checkCSAxisProfiles(int mesg_function)
{
  unsigned i, j;				//Looping
  GalilCSAxis *pCSAxes[MAX_GALIL_AXES] = {0};	//GalilCSAxis list
  char mesg[MAX_GALIL_STRING_SIZE];		//Controller error mesg if begin fail
  int useCSAxis[MAX_GALIL_CSAXES] = {0};	//CSAxis useAxis flag for profile moves
  int useAxis[MAX_GALIL_AXES] = {0};		//Axis useAxis flag for profile moves
  int moveCSMode[MAX_GALIL_AXES] = {0};		//CSAxis moveMode flag for profile moves
  int moveMode[MAX_GALIL_AXES] = {0};		//Axis moveMode flag for profile moves
  string str1, str2;				//String searching
  size_t found;					//String searching

  //Collect all CSAxis instances, moveModes, and useAxes flags
  for (i = 0; i < MAX_GALIL_CSAXES; i++)
     {
     //Retrieve profileUseAxis_ from ParamList for CSAxis
     getIntegerParam(i + MAX_GALIL_AXES, profileUseAxis_, &useCSAxis[i]);
     //Retrieve GalilProfileMoveMode_ from ParamList for CSAxis
     getIntegerParam(i + MAX_GALIL_AXES, GalilProfileMoveMode_, &moveCSMode[i]);
     //Retrieve GalilCSAxis instance
     pCSAxes[i] = getCSAxis(i + MAX_GALIL_AXES);
     }

  //Collect all Axis moveModes, and useAxes flags
  for (i = 0; i < MAX_GALIL_AXES; i++)
     {
     //Retrieve profileUseAxis_ from ParamList for Axis
     getIntegerParam(i, profileUseAxis_, &useAxis[i]);
     //Retrieve GalilProfileMoveMode_ from ParamList for Axis
     getIntegerParam(i, GalilProfileMoveMode_, &moveMode[i]);
     }

  //Loop thru included CSAXis
  //Look for setup problems
  for (i = 0; i < MAX_GALIL_CSAXES; i++)
     {
     //Decide to process this axis, or skip
     if (!useCSAxis[i] || !pCSAxes[i]) continue;
     //Pass reverse axis list to std::string type for processing
     str1 = pCSAxes[i]->revaxes_;
     //Loop thru all motors in this CSAxis
     //Ensure useAxis set true on all motors
     //Ensure motor moveMode matches CSAxis
     for (j = 0; j < strlen(str1.c_str()); j++)
        {
        //Ensure useAxis set true
        if (!useAxis[pCSAxes[i]->revaxes_[j] - AASCII])
           {
           //Update the message
           sprintf(mesg, "%c axis UseAxis flag is set to no, it should be set yes instead", pCSAxes[i]->revaxes_[j]);
           setStringParam(mesg_function, mesg);
           return asynError;
           }
        //Ensure Axis movemode matches the CSAxis
        if (moveMode[pCSAxes[i]->revaxes_[j] - AASCII] != moveCSMode[i])
           {
           //Update the message
           sprintf(mesg, "%c axis moveMode does not match CSAxis %c", pCSAxes[i]->revaxes_[j], pCSAxes[i]->axisName_);
           setStringParam(mesg_function, mesg);
           return asynError;
           }
        }
     //Check other included CSAxis revaxes against this one
     //If any CSAxis in profile share a motor, then fail
     for (j = 0; j < MAX_GALIL_CSAXES; j++)
        {
        //Decide to process this axis, or skip
        if (i == j) continue;
        if (!useCSAxis[j] || !pCSAxes[j]) continue;
        str2 = pCSAxes[j]->revaxes_;
        found = str1.find_first_of(str2);
        if (found != string::npos)
           {
           //Update the message
           sprintf(mesg, "%c axis cannot be shared.  Review axis included in profile", str1[found]);
           setStringParam(mesg_function, mesg);
           return asynError;
           }
        }
     }

   //No motors are shared, all flags match, return success
   return asynSuccess;
}

/**
 * Transform CSAxis profile data to Axis data
 * @return motor driver status code.
 */
asynStatus GalilController::transformCSAxisProfiles()
{
  unsigned i;			//Looping
  GalilCSAxis *pCSAxis;		//GalilCSAxis
  int status = asynSuccess;	//Return status

  //Loop thru CSAxis
  for (i = 0; i < MAX_GALIL_CSAXES; i++)
     {
     //Retrieve GalilCSAxis
     pCSAxis = getCSAxis(i + MAX_GALIL_AXES);
     //Transform the CSAxis profile
     if (pCSAxis)
        status |= pCSAxis->transformCSAxisProfile();
     }

  //Return code
  return (asynStatus)status;
}

//Creates a profile data file
asynStatus GalilController::buildProfileFile()
{
  GalilAxis *pAxis;				//GalilAxis instance
  int nPoints;					//Number of points in profile
  double velocity[MAX_GALIL_AXES];		//Motor velocity for current segment
  double velocityPrev[MAX_GALIL_AXES];		//Motor velocity for previous segment
  double maxAllowedVelocity[MAX_GALIL_AXES];    //Derived from MR VMAX to ensure motor velocities are within limits
  double maxProfileVelocity[MAX_GALIL_AXES];    //The highest velocity for each motor in the profile data
  double maxProfilePosition[MAX_GALIL_AXES];	//Maximum profile position in absolute mode
  double minProfilePosition[MAX_GALIL_AXES];	//Minimum profile position in absolute mode
  double maxProfileAcceleration[MAX_GALIL_AXES];//Maximum profile acceleration in any mode
  double vectorVelocity;			//Segment vector velocity
  double incmove = 0.0;				//Motor incremental move distance
  double firstmove[MAX_GALIL_AXES];	//Used to normalize moves to relative, and prevent big jumps at profile start			
  double apos[MAX_GALIL_AXES];		//Accumulated profile position calculated from integer rounded units (ie. steps/counts)
  double aerr[MAX_GALIL_AXES];		//Accumulated error
  int i, j;			        //Loop counters
  int zm_count;				//Zero segment move counter
  int num_motors;			//Number of motors in trajectory
  int pvtcapable, proftype;		//PVT capable, and profile type (Linear or PVT)
  char message[MAX_GALIL_STRING_SIZE];	//Profile build message
  int useAxis[MAX_GALIL_AXES];		//Use axis flag for profile moves
  int moveMode[MAX_GALIL_AXES];		//Move mode absolute or relative
  char moves[MAX_GALIL_STRING_SIZE];	//Segment move command assembled for controller
  char axes[MAX_GALIL_AXES];		//Motors involved in profile move
  char startp[MAX_GALIL_STRING_SIZE];	//Profile start positions written to file
  char fileName[MAX_FILENAME_LEN];	//Filename to write profile data to
  FILE *profFile;			//File handle for above file
  bool buildOK=true;			//Was the trajectory built successfully
  double mres;				//Motor resolution
  double temp_time;			//Used for timebase calculations in PVT mode

  //Check CSAxis profiles
  //Ensure motors are not shared
  //Ensure useAxis, and moveMode between CSAxes, and 
  //member reverse (real) motors are consistent
  if (checkCSAxisProfiles(profileBuildMessage_))
     return asynError;

  //Transform CSAxis profiles to axis profiles
  if (transformCSAxisProfiles())
     return asynError;
  
  //No axis included yet
  strcpy(axes, "");

  //Start position list for all motors in the profile
  strcpy(startp, "");

  //Retrieve required attributes from ParamList
  getStringParam(GalilProfileFile_, (int)sizeof(fileName), fileName);
  getIntegerParam(profileNumPoints_, &nPoints);
  getIntegerParam(GalilPVTCapable_, &pvtcapable);
  getIntegerParam(GalilProfileType_, &proftype);

  //Check provided fileName
  if (!abs(strcmp(fileName, "")))
	{
	strcpy(message, "Bad trajectory file name");
	return asynError;
	}

  /* Create the profile file */
  profFile =  fopen(fileName, "wt");

  //Write profile type
  if (proftype)
     fprintf(profFile,"PVT\n");
  else
     fprintf(profFile,"LINEAR\n");

  //Zero variables, construct axes, start position, and maxVelocity lists 
  for (i = 0; i < MAX_GALIL_AXES; i++)
	{
	//Retrieve GalilAxis
	pAxis = getAxis(i);
	//Retrieve useAxis from paramList
	getIntegerParam(i, profileUseAxis_, &useAxis[i]);
	//Decide to process this axis, or skip
	if (!useAxis[i] || !pAxis) continue;
	//Initialize accumulated position, and error
	apos[i] = aerr[i] = 0;
	//Initialize velocity
	velocity[i] = velocityPrev[i] = 0;
	//Construct axis list
	sprintf(axes,"%s%c", axes, (char)(i + AASCII));
	//Construct start positions list
	sprintf(startp,"%s%.0lf,", startp, rint(pAxis->profilePositions_[0]));
	//Retrieve the motor maxVelocity in egu
	getDoubleParam(i, GalilMotorVmax_, &maxAllowedVelocity[i]);
	//Retrieve motor resolution
	getDoubleParam(i, motorResolution_, &mres);
	//Calculate velocity in steps
	maxAllowedVelocity[i] = maxAllowedVelocity[i] / mres;
	//Retrieve GalilProfileMoveMode_ from ParamList
	getIntegerParam(i, GalilProfileMoveMode_, &moveMode[i]);
	//Initialize max profile velocity, position, and acceleration
	maxProfileVelocity[i] = maxProfilePosition[i] = maxProfileAcceleration[i] = 0;
	//Initialize min profile position
	minProfilePosition[i] = DBL_MAX;
	}

  //Write axes list
  fprintf(profFile,"%s\n", axes);

  //Write start positions list
  fprintf(profFile,"%s\n", startp);

  //Determine number of motors in profile move
  num_motors = (int)strlen(axes);

  //Calculate motor segment velocities from profile positions, and common time base
  for (i = 0; i < nPoints; i++)
  	{
	//No controller moves assembled yet for this segment
  	strcpy(moves, "");
	//velocity for this segment
	vectorVelocity = 0.0;
	//motors with zero moves for this segment
	zm_count = 0;
	//Calculate motor incremental move distance, and velocity
	for (j = 0; j < MAX_GALIL_AXES; j++)
		{
		//Retrieve GalilAxis
		pAxis = getAxis(j);
		//Decide to process this axis, or skip
		if (!useAxis[j] || !pAxis)
			{
			//Linear mode, add axis relative move separator character ',' as needed
			if ((j < MAX_GALIL_AXES - 1) && !proftype)
				sprintf(moves,  "%s,", moves);
			//Skip the rest, this axis is not in the profile move
			continue;
			}

		if (i == 0)
			{
			//First segment incremental move distance
			firstmove[j] = pAxis->profilePositions_[i];
			//Velocity set to 0 for first increment
			incmove = velocity[j] = 0.0;
			}
		else
			{
			//Segment incremental move distance
			if (i == 1)
				incmove = pAxis->profilePositions_[i] - firstmove[j];
			else
				incmove = (pAxis->profilePositions_[i] - firstmove[j]) - (pAxis->profilePositions_[i-1] - firstmove[j]);
			//Accumulated position calculated using integer rounded positions (units=steps/counts)
			apos[j] += rint(incmove);
			//Accumulated error caused by integer rounding
			aerr[j] = apos[j] - (pAxis->profilePositions_[i] - firstmove[j]);
			//If accumlated error due to rounding greater than 1 step/count, apply correction
			if (fabs(aerr[j]) > 1)
				{
				//Apply correction to segment incremental move distance
				incmove = incmove - aerr[j];
				//Apply correction to accumulated position calculated using integer rounded positions
				apos[j] = apos[j] - aerr[j];
				}
			//Calculate required velocity for this motor given move distance and time
			velocity[j] = incmove / profileTimes_[i];
			}

		//Retrieve motor resolution
		getDoubleParam(j, motorResolution_, &mres);

		//Check profile velocity less than mr vmax for this motor
		if (fabs(velocity[j]) > fabs(maxAllowedVelocity[j]))
			{
			sprintf(message, "Seg %d: Velocity too high motor %c %2.2f > %2.2f, increase time, check profile", 
                                          i,  pAxis->axisName_, fabs(velocity[j]*mres), maxAllowedVelocity[j]*mres);
			buildOK = false;
			}

		//Find max profile velocity for this motor
		if (fabs(velocity[j]*mres) > maxProfileVelocity[j])
			maxProfileVelocity[j] = fabs(velocity[j]*mres);

		//Find max profile position for this motor
		if ((pAxis->profilePositions_[i]*mres) > maxProfilePosition[j])
			 maxProfilePosition[j] = pAxis->profilePositions_[i]*mres;

		//Find min profile position for this motor
		if ((pAxis->profilePositions_[i]*mres) < minProfilePosition[j])
			 minProfilePosition[j] = pAxis->profilePositions_[i]*mres;

		//Find max profile acceleration for this motor
		if (fabs((velocity[j] - velocityPrev[j])*mres)/profileTimes_[i] > maxProfileAcceleration[j])
			maxProfileAcceleration[j] = fabs((velocity[j] - velocityPrev[j])*mres)/profileTimes_[i];

		//Check position against software limits
		if (pAxis->profilePositions_[i] < pAxis->lowLimit_ || 
		    pAxis->profilePositions_[i] > pAxis->highLimit_)
			{
			//Only if move mode = Absolute
			if (moveMode[j])
				{
				sprintf(message, "Motor %c position beyond soft limits in segment %d", pAxis->axisName_, i);
				buildOK = false;
				}
			}

		if (!proftype)
			{
			//Sum segment vector velocity for non zero move increments
			if (rint(incmove) != 0)
				{
				//Calculate linear mode velocity
				//Add this motors' contribution to segment vector velocity
				vectorVelocity += pow(velocity[j], 2);
				}
			//Store motor incremental move distance for this segment
			sprintf(moves, "%s%.0lf", moves, rint(incmove));
			//Add axis relative move separator character ',' as needed
			if (j < MAX_GALIL_AXES - 1)
				sprintf(moves,  "%s,", moves);
			}
		else
			{
			//PVT mode
			sprintf(moves, "%s%lf %.0lf,%.0lf,%.0lf\n", moves, profileTimes_[i], rint(incmove), velocity[j], rint(profileTimes_[i]*1000.0));
			//Check timebase is multiple of 2ms
			temp_time = profileTimes_[i] * 1000.0;
			//PVT has 2 sample minimum
			if ((int)(rint(temp_time)) % 2 != 0)
				{
				sprintf(message, "Profile time base must be multiple of 2ms in PVT mode");
				buildOK = false;
				}
			}
		//Detect zero moves in this segment
		zm_count =  (rint(incmove) == 0) ? zm_count+1 : zm_count;
		//Store current segment velocity for this motor
		velocityPrev[j] = velocity[j];
		}

	//Check for zero move segments
	if (zm_count == num_motors && i != 0)
		{
		sprintf(message, "Seg %d: Vector zero move distance, reduce time, add motors, and check profile", i);
		if (!num_motors)
			strcpy(message, "No motors in profile, add motors");
		buildOK = false;
		}

	if (!proftype)
		{
		//Linear mode
		//Determine vector velocity for this segment
		vectorVelocity = sqrt(vectorVelocity);
		//Check for segment too short error
		if (rint(vectorVelocity) == 0 && i != 0)
			{
			sprintf(message, "Seg %d: Vector velocity zero, reduce time, add motors, and check profile", i);
			buildOK = false;
			}
		//Trim trailing ',' characters from moves string
		for (j=(int)strlen(moves)-1; j>0; j--)
			{
			if (moves[j] != ',')
				{
				//Terminate moves string
				moves[j+1] = '\0';
				break;
				}
			}
		//Add segment velocity
		sprintf(moves, "%s<%.0lf", moves, rint(vectorVelocity));
		}

	//Add second segment and above
	//First segment is the "relative" offset or start position for the profile
	//This is done to prevent jumps in the motor
	if (i > 0)
		{
		//Write the segment command to profile file
		if (proftype) //PVT
			fprintf(profFile,"%s", moves);
		else	//Linear
			fprintf(profFile,"%lf %s\n", profileTimes_[i], moves);
		}
  	}
  
  //Profile written to file now close the file
  fclose(profFile);

  //Check, and if neccessary restore GalilAxis original profile
  //data after CSAxis profiles have been built
  for (i = 0; i < MAX_GALIL_AXES; i++)
     {
     //Retrieve GalilAxis
     pAxis = getAxis(i);
     //Decide to process this axis, or skip
     if (!pAxis) continue;
     pAxis->restoreProfileData();
     }

  //Build failed.  
  if (!buildOK)
	{
	//Delete profile file because its not valid
	remove(fileName);
	//Update build message
  	setStringParam(profileBuildMessage_, message);
	return asynError;
	}
  else
	{
	//Update profile ParamList attributes if buildOK
	for (j=0; j<MAX_GALIL_AXES; j++)
		{
		//Retrieve GalilAxis
		pAxis = getAxis(j);
		//Decide to process this axis, or skip
		if (!useAxis[j] || !pAxis) continue;
		pAxis->setDoubleParam(GalilProfileMinPosition_, minProfilePosition[j]);
		pAxis->setDoubleParam(GalilProfileMaxPosition_, maxProfilePosition[j]);
		pAxis->setDoubleParam(GalilProfileMaxVelocity_, maxProfileVelocity[j]);
		pAxis->setDoubleParam(GalilProfileMaxAcceleration_, maxProfileAcceleration[j]);
		pAxis->callParamCallbacks();
		}
	}

  return asynSuccess;
}

//Overridden as we need CSAxis, and calculatedPositions_ (function=ProfileCalculated_) waveform
/* These are the functions for profile moves */
/** Initialize a profile move of multiple axes. */
asynStatus GalilController::initializeProfile(size_t maxProfilePoints)
{
  int axis;
  GalilAxis *pAxis;
  GalilCSAxis *pCSAxis;
  
  maxProfilePoints_ = maxProfilePoints;
  if (profileTimes_) free(profileTimes_);
  profileTimes_ = (double *)calloc(maxProfilePoints, sizeof(double));
  for (axis = 0; axis < MAX_GALIL_AXES + MAX_GALIL_CSAXES; axis++) 
     {
     if (axis < MAX_GALIL_AXES)
         {
         pAxis = getAxis(axis);
         if (pAxis)
            pAxis->initializeProfile(maxProfilePoints);
         }
     else
         {
         pCSAxis = getCSAxis(axis);
         if (pCSAxis)
            pCSAxis->initializeProfile(maxProfilePoints);
         }
     }
  return asynSuccess;
}

/* Function to build, install and verify trajectory */ 
asynStatus GalilController::buildProfile()
{
  int status;				//asynStatus
  char message[MAX_GALIL_STRING_SIZE];	//Profile build message
  static const char *functionName = "buildProfile";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);
            
  //Call the base class method which will build the time array if needed
  asynMotorController::buildProfile();

  //Update profile build status
  strcpy(message, "");
  setStringParam(profileBuildMessage_, message);
  setIntegerParam(profileBuildState_, PROFILE_BUILD_BUSY);
  setIntegerParam(profileBuildStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();

  //Build profile data
  status = buildProfileFile();

  //Update profile build state
  setIntegerParam(profileBuildState_, PROFILE_BUILD_DONE);
  //Update profile build status
  if (status)
	setIntegerParam(profileBuildStatus_, PROFILE_STATUS_FAILURE);
  else
	setIntegerParam(profileBuildStatus_, PROFILE_STATUS_SUCCESS);
  callParamCallbacks();

  return asynSuccess;
}

/* Function to execute trajectory */ 
asynStatus GalilController::executeProfile()
{
  epicsEventSignal(profileExecuteEvent_);
  return asynSuccess;
}

/* C Function which runs the profile thread */ 
static void GalilProfileThreadC(void *pPvt)
{
  GalilController *pC = (GalilController*)pPvt;
  pC->profileThread();
}

/* Function which runs in its own thread to execute profiles */ 
void GalilController::profileThread()
{
  while (true) {
    epicsEventWait(profileExecuteEvent_);
    runProfile();
  }
}

asynStatus GalilController::abortProfile()
{
  //Request the thread that buffers/executes the profile to abort the process
  profileAbort_ = true;

  //Stop the profile axis
  if (strcmp(profileAxes_, "") != 0)
    {
    //Stop the axes
    sprintf(cmd_, "ST %s", profileAxes_);
    //Write setting to controller
    sync_writeReadController();
    }

  return asynSuccess;
}

/* C Function which runs the array upload thread */ 
static void GalilArrayUploadThreadC(void *pPvt)
{
  GalilController *pC = (GalilController*)pPvt;
  pC->arrayUploadThread();
}

/* Array upload function runs in its own thread */ 
void GalilController::arrayUploadThread()
{
  while (true) {
    epicsEventWait(arrayUploadEvent_);
    arrayUpload();
  }
}

//Execute prem for motor list
//Obtain lock before calling
void GalilController::executePrem(const char *axes)
{
   int i;
   int axisNo;			//Axis number
   GalilAxis *pAxis;		//GalilAxis
    
   //Iterate thru motor list and execute prem command for each
   for (i = 0; axes[i] != '\0'; i++)
      {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve axis instance
      pAxis = getAxis(axisNo);
      //Execute prem for this axis
      if (pAxis)
         pAxis->executePrem();
      }
}

//Execute motor auto on and brake off for motor list
//Obtain lock before calling
void GalilController::executeAutoOnBrakeOff(const char *axes)
{
   int i;			//Looping
   int autoonoff;		//Auto on/off setting
   int motoroff;		//Motor off status
   int axisNo;			//Axis number
   GalilAxis *pAxis;		//GalilAxis
   double ondelay;		//GalilAxis auto on delay
   double largest_ondelay = 0.0;//Largest auto on delay in motor list
   int autobrake;		//Brake auto disable/enable setting
   int brakeoff;		//Motor brake off status
   int brakeport;		//Brake digital out port
   bool workdone = false;	//Did work to turn a motor on, or brake off

   //Iterate thru motor list and execute AutoOn for each axis
   for (i = 0; axes[i] != '\0'; i++)
      {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve axis instance
      pAxis = getAxis(axisNo);
      //Execute AutoOnBrakeOff for this axis
      if (pAxis)
         {
         //Retrieve Auto power on/off feature status
         getIntegerParam(pAxis->axisNo_, GalilAutoOnOff_, &autoonoff);
         //Retrieve Auto brake on/off feature status
         getIntegerParam(pAxis->axisNo_, GalilAutoBrake_, &autobrake);
         //Retrieve brake digital output port
         getIntegerParam(pAxis->axisNo_, GalilBrakePort_, &brakeport);
         //Retrieve axis auto on delay
         getDoubleParam(pAxis->axisNo_, GalilAutoOnDelay_, &ondelay);
         //Take note of largest on delay found in axes list
         if (ondelay > largest_ondelay)
            largest_ondelay = ondelay;
         //Query motor off status direct from controller
         sprintf(cmd_, "MG _MO%c", pAxis->axisName_);
         sync_writeReadController();
         motoroff = atoi(resp_);
         //Case where auto on delay greater than auto off delay
         //Block auto off function
         pAxis->autooffAllowed_ = false;
         //Execute auto motor on if feature enabled
         if (motoroff && autoonoff)
            {
            pAxis->setClosedLoop(true);	//Turn on this axis
            workdone = true;
            }

         //Query brake status direct from controller
         if (brakeport > 0)
            {
            sprintf(cmd_, "MG @OUT[%d]\n", brakeport);
            sync_writeReadController();
            brakeoff = atoi(resp_);
            //Execute auto brake off if feature enabled
            if (!brakeoff && autobrake)
               {
               pAxis->setBrake(false);	//brake off command
               workdone = true;
               }
            }
         }
      }

   //Wait the longest auto on time found in list
   if (workdone)
      {
      if (largest_ondelay > 0.035)
         {
         //Safe to unlock for ondelay now all axis have autooffAllowed_ = false;
         unlock();
         //Wait auto on delay
         epicsThreadSleep(largest_ondelay);
         //Get the lock
         lock();
         }
      else
         epicsThreadSleep(largest_ondelay);
     }

  //Allow auto off again now we have lock again
  for (i = 0; axes[i] != '\0'; i++)
      {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve axis instance
      pAxis = getAxis(axisNo);
      //allow auto off now we have lock again
      if (pAxis)
         {
         //Reset stop timer for auto off
         pAxis->stop_begint_ = pAxis->stop_nowt_;
         pAxis->autooffSent_ = false;
         //Allow auto off now we have lock
         pAxis->autooffAllowed_ = true;
         }
      }
}

/* For profile moves.  Convenience function to move motors to start or stop them moving to start
*/
asynStatus GalilController::motorsToProfileStartPosition(FILE *profFile, double startp[], bool move = true)
{
  GalilAxis *pAxis;			//GalilAxis
  int i;				//Axis looping
  int axisNo;				//Axis number
  int moveMode[MAX_GALIL_AXES];  	//Move mode absolute or relative
  int moving;				//Axis moving status
  int deferredMode;			//Backup of current deferredMode
  double accl, velo, mres;		//Required mr attributes
  double velocity, acceleration;	//Used to move motors to start
  char message[MAX_GALIL_STRING_SIZE];	//Profile execute message
  int status = asynSuccess;

  if (move)
     {
     //Update status
     strcpy(message, "Moving motors to start position and buffering profile data...");
     setStringParam(profileExecuteMessage_, message);
     callParamCallbacks();
     //Set deferred moves true
     setDeferredMoves(true);
     //Retrieve deferred moves mode
     getIntegerParam(GalilDeferredMode_, &deferredMode);
     //We must use sync start only mode
     //To avoid interferring with profile download
     setIntegerParam(GalilDeferredMode_, 0);
     }

  //If move mode absolute then set motor setpoints to profile start position
  //or stop motors moving to start position
  for (i = 0; profileAxes_[i] != '\0'; i++)
     {
     //Determine the axis number mentioned in profFile
     axisNo = profileAxes_[i] - AASCII;
     if (move) //Read profile start positions from file
        fscanf(profFile, "%lf,", &startp[axisNo]);
     //Retrieve GalilProfileMoveMode_ from ParamList
     getIntegerParam(axisNo, GalilProfileMoveMode_, &moveMode[axisNo]);
     //If moveMode = Relative skip move to start
     if (!moveMode[axisNo]) continue;
     //Retrieve axis instance
     pAxis = getAxis(axisNo);
     //Skip axis if not instantiated
     if (!pAxis) continue;
     //Retrieve needed mr attributes
     getDoubleParam(axisNo, GalilMotorAccl_, &accl);
     getDoubleParam(axisNo, GalilMotorVelo_, &velo);
     getDoubleParam(axisNo, motorResolution_, &mres);
     //Calculate velocity and acceleration in motor steps
     velocity = fabs(velo/mres);
     acceleration = fabs(velocity/accl);
     //If moveMode = Absolute, then set motor setpoints to profile start position
     if (move)
        {
        //Check motor record settings before move
        status = pAxis->checkMRSettings(false, pAxis->axisName_);
        //Check motor interlock before move
        status |= pAxis->beginCheck("motorsToProfileStartPosition", 100, false);
        if (!status)
           {
           //Set motor setpoints, but dont do the move (movesDeferred = true)
           if (!pAxis->moveThruMotorRecord(startp[axisNo], velocity, acceleration, false))
              {
              //Move success
              //Release the lock
              //This releases synchronrous poller
              //And GalilAxis::move is called (since this is a separate thread)
              unlock();
              //Wait till axis moving status becomes true due to deferred move
              moving = 0;
              while (!moving)
                 {
                 //Determine moving status
                 getIntegerParam(pAxis->axisNo_, motorStatusMoving_, &moving);
                 if (!moving)
                    epicsThreadSleep(.001);
                 }
              //Take the lock before proceeding
              lock();
              }
           }
        }
     else//Stop motor moving to start, prevent backlash, retries
        pAxis->stop(acceleration);
     if (status)
        {
        //Update profile execute message
        sprintf(message, "Failed to move profile motor %c toward start position", pAxis->axisName_);
        setStringParam(profileExecuteMessage_, message);
        //break from the loop
        break;
        }
     }//For

  if (move)
     {
     //Move file pointer down a line
     fscanf(profFile, "\n");
     //Restore previous deferredMode
     setIntegerParam(GalilDeferredMode_, deferredMode);
     //Release deferred moves here
     setDeferredMoves(false);
     }
  else
     {
     //Release lock
     unlock();
     //Wait for motion to complete
     while (anyMotorMoving())
        epicsThreadSleep(.001);
     //Obtain the lock
     lock();
     }

  return (asynStatus)status;
}

/* Convenience function to begin profile
   Called after filling the linear buffer
*/
asynStatus GalilController::beginProfileMotion(int coordsys, char coordName)
{
   char mesg[MAX_GALIL_STRING_SIZE];
   asynStatus status = asynSuccess;

   //Begin profile
   if (!profileType_)
      status = beginLinearGroupMotion(coordsys, coordName, profileAxes_, true);
   else
      status = beginPVTProfileMotion();

   //Set message
   if (status)
      {
      //Start fail
      strcpy(mesg, "Profile start failed...\n");
      //Store message in ParamList
      setStringParam(profileExecuteMessage_, mesg);
      }
   else
      {
      //Start success
      setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);
      strcpy(mesg, "Profile executing...");
      //Store message in ParamList
      setStringParam(profileExecuteMessage_, mesg);
      }

   //Post message
   callParamCallbacks();
   //Return status
   return status;
}

/* Convenience function to begin PVT group
   Called after filling the PVT buffer
*/
asynStatus GalilController::beginPVTProfileMotion()
{
  double begin_time = 0;		//Time taken to begin
  int pmoving = 0;			//Profile moving status
  unsigned i;				//Looping
  asynStatus status = asynSuccess;	//Error status

  //Execute motor auto on and brake off function
  executeAutoOnBrakeOff(profileAxes_);

  //Execute motor record prem
  executePrem(profileAxes_);

  //Set accel/decel for profileAxes_
  for (i = 0; profileAxes_[i] != '\0'; i++)
     {
     //Set acceleration/deceleration
     sprintf(cmd_, "AC%c=%ld;DC%c=%ld", profileAxes_[i], maxAcceleration_, profileAxes_[i], maxAcceleration_);
     sync_writeReadController();
     }

  //Begin the move
  //Get time when attempt motor begin
  epicsTimeGetCurrent(&begin_begint_);
  sprintf(cmd_, "BT %s", profileAxes_);
  if (sync_writeReadController() == asynSuccess)
     {
     unlock();
     while (!pmoving)
        {
        epicsThreadSleep(.001);
        epicsTimeGetCurrent(&begin_nowt_);
        //Calculate time begin has taken so far
        begin_time = epicsTimeDiffInSeconds(&begin_nowt_, &begin_begint_);
        if (begin_time > BEGIN_TIMEOUT)
           {
           status = asynError;
           break;  //Timeout, give up
           }

        //Check profile abort
        if (profileAbort_)
           break; //User pressed profile abort

        //Retrieve coordinate system moving status as set by poll thread
        getIntegerParam(profileAxes_[0] - AASCII, motorStatusMoving_, &pmoving);
        }
     lock();
     }
  else  //Controller gave error at begin
     status = asynError;

  //Return success
  return status;
}

/* Convenience function to begin linear group using specified coordinate system
   Called after filling the linear buffer
*/
asynStatus GalilController::beginLinearGroupMotion(int coordsys, char coordName, const char *axes, bool profileAbort = false)
{
  double begin_time = 0;		//Time taken to begin
  int csmoving = 0;			//Coordsys moving status
  asynStatus status = asynSuccess;	//Error status

  //Execute motor auto on and brake off function
  executeAutoOnBrakeOff(axes);

  //Execute motor record prem
  executePrem(axes);

  getIntegerParam(coordsys, GalilCoordSysMoving_, &csmoving);

  //Begin the move
  //Get time when attempt motor begin
  epicsTimeGetCurrent(&begin_begint_);
  sprintf(cmd_, "BG %c", coordName);
  if (sync_writeReadController() == asynSuccess)
     {
     unlock();
     while (!csmoving)
        {
        epicsThreadSleep(.001);
        epicsTimeGetCurrent(&begin_nowt_);
        //Calculate time begin has taken so far
        begin_time = epicsTimeDiffInSeconds(&begin_nowt_, &begin_begint_);
        if (begin_time > BEGIN_TIMEOUT)
           {
           status = asynError;
           break;  //Timeout, give up
           }

        //If caller cares about profile abort then check it
        if (profileAbort_ && profileAbort)
           break; //User pressed profile abort

        //Retrieve coordinate system moving status as set by poll thread
        getIntegerParam(coordsys, GalilCoordSysMoving_, &csmoving);
        }
     lock();
     }
  else  //Controller gave error at begin
     status = asynError;

  //Return success
  return status;
}

/*
 prepRunProfile - Preparation sequence for both Linear, and PVT profiles
 profFile [in] - Trajectory file
 axes[out] - axes in the profile
 startp[out] - profile start positions for each axis
*/
asynStatus GalilController::prepRunProfile(FILE **profFile, int *coordsys, char *coordName, double startp[])
{
  const char *functionName = "prepRunProfile";
  unsigned i;					//Looping
  asynStatus status = asynSuccess;		//Return value
  GalilAxis *pAxis;				//GalilAxis
  char mesg[MAX_GALIL_STRING_SIZE]= "";		//Profile execute message
  char fileName[MAX_FILENAME_LEN];		//Filename to read profile data from
  char profileType[MAX_GALIL_STRING_SIZE];	//String read from trajectory file that represents profile type (ie. LINEAR or PVT)
  int pvtcapable;				//Controller PVT capable status

  //Retrieve pvt capable status
  getIntegerParam(GalilPVTCapable_, &pvtcapable);

  //Retrieve currently selected coordinate system 
  getIntegerParam(GalilCoordSys_, coordsys);

  //Selected coordinate system name
  *coordName = (*coordsys == 0 ) ? 'S' : 'T';

  //Profile has not been aborted
  profileAbort_ = false;

  //Check CSAxis profiles
  //Ensure motors are not shared
  //Ensure useAxis, and moveMode between CSAxes, and 
  //member reverse (real) motors are consistent
  status = checkCSAxisProfiles(profileExecuteMessage_);
  
  //Open the profile file
  if (!status)
     {
     //Update execute profile status
     setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
     setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
     callParamCallbacks();
     //Retrieve trajectory filename from ParamList
     getStringParam(GalilProfileFile_, (int)sizeof(fileName), fileName);
     //Open trajectory file
     if ((*profFile =  fopen(fileName, "rt")) == NULL)
        {
        //Assemble error message
        strcpy(mesg, "Can't open trajectory file\n");
        status = asynError;
        }
     }

  //File opened ok?
  if (!status)
     {
     //Read file header, and grab profile type
     fscanf(*profFile, "%s\n", profileType);
     //Determine profile type LINEAR or PVT type
     profileType_ = strcmp(profileType, "PVT") == 0 ? 1 : 0;
     //Determine which motors are involved
     fscanf(*profFile, "%s\n", profileAxes_);
     //Loop through the axes list
     //Ensure all motors are enabled
     for (i = 0; profileAxes_[i] != '\0'; i++)
        {
        //Retrieve axis specified in axes list
        pAxis = getAxis(profileAxes_[i] - AASCII);
        if (!pAxis) continue;
        //Check motors are good to go.  Supply arbitary velocity here (100)
        if (pAxis->beginCheck(functionName, 100)) //Show disabled message in profile message area
           {
           //Assemble error message
           sprintf(mesg, "%c not ready, check controller message", pAxis->axisName_);
           status = asynError;
           break;
           }
        }
     }

  //Check profileType_ against controller capability
  if (profileType_ && !pvtcapable && !status)
     {
     strcpy(mesg, "Controller is not PVT capable");
     status = asynError;
     }

  //Setup linear profiles
  if (!profileType_ && !status)
    {
    //Linear profile
    //Set vector acceleration/decceleration
    sprintf(cmd_, "VA%c=%ld;VD%c=%ld", *coordName, maxAcceleration_, *coordName, maxAcceleration_);
    sync_writeReadController();

    //Clear any segments in the coordsys buffer
    sprintf(cmd_, "CS %c", *coordName);
    sync_writeReadController();

    //Update coordinate system motor list at record layer
    setStringParam(*coordsys, GalilCoordSysMotors_, profileAxes_);

    //Set linear interpolation mode and include motor list provided
    sprintf(cmd_, "LM %s", profileAxes_);
    sync_writeReadController();
    }
  else if (!status)
    {
    //PVT profile
    for (i = 0; profileAxes_[i] != '\0'; i++)
       {
       //Clear any segments in the pvt buffer(s)
       sprintf(cmd_, "PV%c=0,0,-1", profileAxes_[i]);
       sync_writeReadController();
       }
    }
  
  //Display any error message, and return error
  if (status)
     {
     //Set profile message and status
     setStringParam(profileExecuteMessage_, mesg);
     setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);
     setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
     callParamCallbacks();
     //Close the profile file
     if (*profFile != NULL)
        fclose(*profFile); //Close the trajectory file
     }

  //Return result
  return status;
}

void GalilController::profileGetSegsMoving(int profStarted, int coordsys, int *moving, int *segprocessed)
{
   if (profileType_)
      {
      //PVT profile
      //Segments processed
      if (profStarted)
         {
         sprintf(cmd_, "MG _BT%c", profileAxes_[0]); //Use first axis as measure of segs processed
         sync_writeReadController();
         *segprocessed = atoi(resp_);
         //Profile moving status
         getIntegerParam(profileAxes_[0] - AASCII, motorStatusMoving_, moving);
         //PVT end takes 1 segment, compensate in final segment count
         if (!*moving)
            *segprocessed -= 1;
         }
      }
   else
      {
      //Linear profile
      //Segments processed
      getIntegerParam(coordsys, GalilCoordSysSegments_, segprocessed);
      //Profile moving status
      getIntegerParam(coordsys, GalilCoordSysMoving_, moving);
      }

   //Update profile current point in ParamList
   setIntegerParam(profileCurrentPoint_, *segprocessed);
   callParamCallbacks();
}

/* Function to run trajectory.  It runs in a dedicated thread, so it's OK to block.
 * It needs to lock and unlock when it accesses class data. */ 
asynStatus GalilController::runProfile()
{
  int segsent;				//Segments loaded to controller so far
  char moves[MAX_GALIL_STRING_SIZE];	//Segment move command assembled for controller
  char message[MAX_GALIL_STRING_SIZE];	//Profile execute message
  vector<double> profileTimes;		//Profile time base read from file
  double profileTime;			//Profile time for segment being buffered to controller
  unsigned i;				//Axes index for PVT profiles
  unsigned nmotors = 0;			//Number of axis in PVT profile
  bool profStarted = false;		//Has profile execution started
  bool atStart = false;			//Have the motors arrived at the start position
  int segprocessed;			//Segments processed by coordsys
  int pmoving;				//Moving status of the profile
  int coordsys;				//Coordinate system S(0) or T(1)
  char coordName;			//Coordinate system S or T
  int maxsegs;				//Maximum number of segments
  int startsegs;			//Segments to buffer before starting
  int retval = 1;			//Return value from file read
  bool bufferNext = true;		//Controller ready to buffer next segment
  double startp[MAX_GALIL_AXES];	//Motor start positions from file
  FILE *profFile = NULL;		//File handle to trajectory file
  asynStatus status = asynSuccess;	//Return value

  //Called without lock, and we need it to call sync_writeReadController
  lock();

  //prepare the profile
  //Retrieve profile file pointer, profile type, coordsys (linear only), coordName (linear only)
  //Retrieve profile axes list, and start positions from file
  if (prepRunProfile(&profFile, &coordsys, &coordName, startp))
     {
     //Unlock the mutex
     unlock();
     //A fail in prepRunProfile is fatal
     return asynError;
     }

  //Move motors to start position, and return start position values here
  status = motorsToProfileStartPosition(profFile, startp);

  unlock();

  //No segments sent to controller just yet
  segsent = 0;
  //Number of segments processed
  segprocessed = 0;
  //Default maximum number of segments
  maxsegs = MAX_LINEAR_SEGMENTS;

  //Initialize local variables for PVT profile
  if (profileType_)
     {
     //Default maximum number of segments
     maxsegs = MAX_PVT_SEGMENTS;
     //Initialize PVT variables
     i = 0;
     nmotors = (unsigned)strlen(profileAxes_);
     }

  //Default number of segments to buffer before start
  startsegs = maxsegs;

  //Execute the profile
  //Loop till file downloaded to buffer, or error, or abort
  //Note Linear is 1 loop per segment
  //PVT is nmotor loops per segment
  while (retval != EOF && !status && !profileAbort_)
	{
	if (bufferNext)
		{
		//Read the segment
		retval = fscanf(profFile, "%lf %s\n", &profileTime, moves);
		//Store profile time
		profileTimes.push_back(profileTime);
		}

	lock();
	//Process segment if didnt hit EOF
	if (retval != EOF)
		{
		//Retrieve moving status, and segments processed
		profileGetSegsMoving(profStarted, coordsys, &pmoving, &segprocessed); 

		//Case where profile has started, but then stopped
		if ((profStarted && !pmoving))
			{
			unlock();
			//Give other threads a chance to get the lock
			epicsThreadSleep(0.001);
			break;	//break from loop
			}

		//Check if motors arrived at start position
		if (!profStarted && !atStart)
			{
			//Abort if motor stopped and not at start position, or limit
			if (!allMotorsMoving(profileAxes_))
				status = motorsAtStart(startp) ? asynSuccess : asynError;
			if (!anyMotorMoving())
				{
				status = motorsAtStart(startp) ? asynSuccess : asynError;
				atStart = (status) ? false : true;
				}
			}

		//Buffer next segment if no error, user hasnt pressed abort, there is buffer space
		if (!status && !profileAbort_ && bufferNext)
			{
			//Proceed to send next segment to controller
			if (!profileType_)	//LINEAR
				sprintf(cmd_, "LI %s", moves);
			else		//PVT
				sprintf(cmd_, "PV%c=%s", profileAxes_[i], moves);
			status = sync_writeReadController();
			if (status)
				{
				unlock();
				epicsThreadSleep(.2);
				lock();
				abortProfile();
				sprintf(message, "Error downloading segment %d", segsent + 1);
				setStringParam(profileExecuteMessage_, message);
				}
			else	
				{
				//Increment segments sent to controller
				if (!profileType_) //Linear segment
					segsent++;
				else
					{
					//PVT profile
					//Increment PVT axis
					i++;
					//Has a complete segment been sent?
					if (i >= nmotors)
						{
						//Full segment has been sent
						segsent++;
						//Again back to first PVT axis
						i = 0;
						}
					}
				}
			}

		//Ensure segs are being sent faster than can be processed by controller
		if (((segsent - segprocessed) <= 2) && profStarted && !status && !profileAbort_)
			{
			abortProfile();
			unlock();
			epicsThreadSleep(.2);
			lock();
			strcpy(message, "Profile time base too fast\n");
			setStringParam(profileExecuteMessage_, message);
			//break loop
			status = asynError;
			}

		//Check profile start condition
		if ((segsent - segprocessed) >= startsegs && !status && !profileAbort_)
			{
			//Buffered startsegs number of segments
			//Case where motors were moving to start position, and now complete, profile is not started.
			if (!profStarted && atStart && !status && !profileAbort_)
				{
				//Start the profile
				status = beginProfileMotion(coordsys, coordName);
				profStarted = (status) ? false : true;
				}
			}

		//Default bufferNext true before checking buffer
		bufferNext = true;
		//Check buffer, and abort status
		if ((segsent - segprocessed) >= maxsegs && !status && !profileAbort_)
			{
			//Segment buffer is full, and user has not pressed abort

			//Dont buffer next segment	
			//Give time for motors to arrive at start, or
			//Give time for controller to process a segment
			if ((anyMotorMoving() && !profStarted) || profStarted)
				{
				unlock();
				//Sleep for time equal to currently processing segment
				epicsThreadSleep(profileTimes[segprocessed]);
				bufferNext = false;
				lock();
				}
			}
		}

	unlock();
	//Give other threads a chance to get the lock
	epicsThreadSleep(0.001);
	}

  lock();

  //All segments have been sent to controller
  if (!profileType_)
     {
     //End linear interpolation mode
     strcpy(cmd_, "LE");
     sync_writeReadController();
     }
  else
     {
     //End PVT mode
     for (i = 0; profileAxes_[i] != '\0'; i++)
        {
        sprintf(cmd_, "PV%c=0,0,0", profileAxes_[i]);
        sync_writeReadController();
        }
     }

  //Check if motors still moving to start position
  if (!profStarted && !status && !profileAbort_)
	{
	//Pause till motors stop
	while (anyMotorMoving())
		{
		unlock();
		epicsThreadSleep(.01);
		lock();
		}
	
	//Start short profiles that fit entirely in the controller buffer <= MAX_SEGMENTS
	//If motors at start position, begin profile
	if (motorsAtStart(startp))
		{
		status = beginProfileMotion(coordsys, coordName);
		profStarted = (status) ? false : true;
		}
	}
 
  //Profile not started, and motors still moving, stop them
  if (!profStarted && anyMotorMoving())
  	motorsToProfileStartPosition(profFile, startp, false);  //Stop motors moving to start position

  //Finish up
  if (!status)
	{
	if (profStarted)
		{
		pmoving = 1;
		//Loop until stopped, or aborted
		while (pmoving)
			{
			//Retrieve moving status, and segments processed
			profileGetSegsMoving(profStarted, coordsys, &pmoving, &segprocessed);
			//Restrict loop frequency
			unlock();
			//Sleep for time equal to currently processing segment
			epicsThreadSleep(profileTimes[segprocessed]);
			lock();
			}
		}

	//Were all segments processed by controller
	if (segprocessed == segsent)
		{
		setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_SUCCESS);
		strcpy(message, "Profile completed successfully");
		setStringParam(profileExecuteMessage_, message);
		}
	else
		{
		//Not all segments were processed
		setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);
		if (profileAbort_)
			strcpy(message, "Profile was stopped by user");
		else
			strcpy(message, "Profile was stopped by limit switch or other motor/encoder problem");
		setStringParam(profileExecuteMessage_, message);
		}
	}
  else  //Coordinate system didnt start, or aborted by user
	setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);

  //Update status
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);
  callParamCallbacks();

  if (profFile != NULL)
     fclose(profFile); //Close the trajectory file

  unlock();

  return asynSuccess;
}

/**
 * Perform a deferred move (a coordinated group move) on all the axes in a group.
 * Motor start and stop times are synchronized regardless of any kinematics
 * Kinematics for position are obeyed.  Velocity, acceleration may not match kinematics to force synchronise stop time
 * @param coordsys - Coordinate system to use
 * @param axes - The list of axis/motors in the coordinate system (eg. "ABCD")
 * @param moves - The comma separated list of the relative moves for each axis/motor in the coordinate system (eg 1000,,-1000,1000)
 * @param acceleration - The vector acceleration for the coordsys
 * @param velocity - The vector velocity for the coordsys
 * @return motor driver status code.
 */
asynStatus GalilController::beginSyncStartStopMove(int coordsys, char *axes, char *moves, double acceleration, double velocity)
{
  const char *functionName = "beginSyncStartStopMove";
  GalilAxis *pAxis;		//GalilAxis pointer
  char coordName;		//Coordinate system name
  asynStatus status;		//Result
  unsigned index;		//looping
  char mesg[MAX_GALIL_STRING_SIZE];	//Controller error mesg if begin fail

  //Selected coordinate system name
  coordName = (coordsys == 0 ) ? 'S' : 'T';

  //Set the specified coordsys on controller
  sprintf(cmd_, "CA %c", coordName);
  sync_writeReadController();

  //Clear any segments in the coordsys buffer
  sprintf(cmd_, "CS %c", coordName);
  sync_writeReadController();

  //Update coordinate system motor list at record layer
  setStringParam(coordsys, GalilCoordSysMotors_, axes);

  //Set linear interpolation mode and include motor list provided
  sprintf(cmd_, "LM %s", axes);
  sync_writeReadController();

  //Set vector acceleration/decceleration
  sprintf(cmd_, "VA%c=%.0lf;VD%c=%.0lf", coordName, acceleration, coordName, acceleration);
  sync_writeReadController();

  //Set vector velocity
  sprintf(cmd_, "VS%c=%.0lf", coordName, velocity);
  sync_writeReadController();
 
  //Specify 1 segment
  sprintf(cmd_, "LI %s", moves);
  sync_writeReadController();

  //End linear mode
  sprintf(cmd_, "LE");
  sync_writeReadController();

  //move the coordinate system
  status = beginLinearGroupMotion(coordsys, coordName, axes);

  if (status)
     {
     sprintf(mesg, "%s begin failure coordsys %c", functionName, coordName);
     //Set controller error mesg monitor
     setCtrlError(mesg);
     status = asynError;
     }

  //Loop through the axes list for this coordinate system
  //Turn off deferredMove_ for these axis as coordinated move has been started
  for (index = 0; axes[index] != '\0'; index++)
    {
    //Retrieve axis specified in axes list
    pAxis = getAxis(axes[index] - AASCII);
    //Process or skip
    if (!pAxis) continue;
    //Set flag
    pAxis->deferredMove_ = false;
    }

  return status;
}

//Prepare Sync start and stop mode moves
//Coordinates groups of motors
//Both start and stop times of motors are synchronized
//Uses linear mode to achieve these goals
asynStatus GalilController::prepSyncStartStopMoves(void)
{
  const char *functionName = "prepSyncStartStopMoves";
  GalilAxis *pAxis;			//GalilAxis pointer
  int coordsys;				//Coordinate system looping
  unsigned axis;			//Axis looping
  char axes[MAX_GALIL_AXES];		//Constructed list of axis in the coordinate system
  char moves[MAX_GALIL_STRING_SIZE];	//Constructed comma list of axis relative moves
  double vectorAcceleration;		//Coordinate system acceleration
  double vectorVelocity;		//Coordinate system velocity

  //Loop through coordinate systems, looking for work to perform
  for (coordsys = 0; coordsys < COORDINATE_SYSTEMS; coordsys++) 
     {
     //No work found yet in this coordsys
     strcpy(axes, "");
     strcpy(moves, "");
     //velocity for this segment
     vectorVelocity = 0.0;
     vectorAcceleration = 0.0;

     //Loop through the axis looking for deferredMoves in this coordsys
     for (axis = 0; axis < MAX_GALIL_AXES; axis++)
        {
        //Retrieve the axis
        pAxis = getAxis(axis);
        //Process or skip
        if (!pAxis) continue;
        //Look for deferred move
        if (pAxis->deferredCoordsys_ == coordsys && pAxis->deferredMove_ && pAxis->deferredMode_)
           {
           //Deferred move found in this coordsys
           //Ensure motor is good to go
           if (pAxis->beginCheck(functionName, pAxis->deferredVelocity_, false))
              {
              //Motor has problem, so turn off deferred move for this axis
              pAxis->deferredMove_ = false;
              }
           else
              {
              //Motor is ready to go
              //Store axis in coordinate system axes list
              sprintf(axes, "%s%c", axes, pAxis->axisName_);
              //Store axis relative move
              sprintf(moves, "%s%.0lf", moves, pAxis->deferredPosition_);
              //Sum vector acceleration, velocity for non zero move increments
              if (rint(pAxis->deferredPosition_) != 0)
                 {
                 //Add this motors' contribution to vector acceleration for this segment
                 vectorAcceleration += pow(pAxis->deferredAcceleration_, 2);
                 //Add this motors' contribution to vector velocity for this segment
                 vectorVelocity += pow(pAxis->deferredVelocity_, 2);
                 }
              }
           }
        //Add axis relative move separator character ',' as needed
        if (axis < MAX_GALIL_AXES - 1)
           sprintf(moves,  "%s,", moves);
        }

     //If at least one axis was found with a deferred move in this coordinate system
     //then process coordsys
     if (strcmp(axes, ""))
        {
        //Calculate final vectorVelocity and vectorAcceleration
        vectorVelocity = sqrt(vectorVelocity);
        vectorAcceleration = sqrt(vectorAcceleration);
        vectorVelocity = lrint(vectorVelocity/2.0) * 2;
        vectorAcceleration = lrint(vectorAcceleration/1024.0) * 1024;
        //Start the move
        beginSyncStartStopMove(coordsys, axes, moves, vectorAcceleration, vectorVelocity);
        }
    }

  return asynSuccess;
}

/**
 * Start a group of motors in independent mode simultanously
 * @param [in] maxes - The list of axis/motors that are moved (eg. "ABCD")
 * @param [in] paxes - The list of axis/motors that are prepared for a move (eg. "ABCD")
 * @return motor driver status code.
 */
asynStatus GalilController::beginGroupMotion(char *maxes, char *paxes)
{
  const char *functionName = "beginMotionGroup";
  GalilAxis *pAxis;			//GalilAxis instance
  char mesg[MAX_GALIL_STRING_SIZE];	//Controller mesg
  int moving = 0;			//Moving status
  double begin_time = 0;		//Time taken to begin
  bool fail = false;			//Fail flag
  asynStatus status = asynSuccess;	//Return status
  char allaxes[MAX_GALIL_AXES];		//Move axes, and prepareAxes list concatenated

  //Retrieve 1st motor GalilAxis instance
  pAxis = getAxis(maxes[0] - AASCII);
  if (!pAxis) return asynError;

  //Concatenate list of move axes, and prepare to move axes
  sprintf(allaxes,"%s%s", maxes, paxes);

  //Execute motor auto on and brake off function
  executeAutoOnBrakeOff(allaxes);

  //Execute motor record prem
  executePrem(allaxes);

  //For move axes only, begin the move
  //Get time when attempt motor begin
  epicsTimeGetCurrent(&begin_begint_);
  sprintf(cmd_, "BG %s", maxes);
  if (sync_writeReadController() == asynSuccess)
     {
     unlock();
     while (!moving)//Allow time for motion to begin
        {
        //Retrieve moving status
        moving = allMotorsInMotion(maxes);
        if (!moving)
           {
           epicsThreadSleep(.001);
           epicsTimeGetCurrent(&begin_nowt_);
           //Calculate time begin has taken so far
           begin_time = epicsTimeDiffInSeconds(&begin_nowt_, &begin_begint_);
           if (begin_time > BEGIN_TIMEOUT)
              {
              fail = true;  //Time is up, give up
              break;
              }
           }
        }
     lock();
     }
  else//Controller gave error at begin
     fail = true;

  if (fail)
     {
     sprintf(mesg, "%s begin failure %s", functionName, maxes);
     //Set controller error mesg monitor
     setCtrlError(mesg);
     status = asynError;
     }

  return status;
}

/** Returns true if all motors in the provided list are moving
  * Provides raw moving status direct from controller
  * This function is not effected by deferredMoves true
  * Called from beginGroupMotion
  * \param[in] Motor list
  */
bool GalilController::allMotorsInMotion(char *axes)
{
  GalilAxis *pAxis;	//GalilAxis instance
  int i;		//Looping

  //Look through motor list, if any not moving return false
  for (i = 0; axes[i] != '\0'; i++)
	{
	//Retrieve the axis
	pAxis = getAxis(axes[i] - AASCII);
	//Process or skip
	if (!pAxis) continue;
	//Determine moving status
        if (!pAxis->inmotion_)
           return false;
        }

  //All the motors were moving
  return true;
}

/**
 * Perform a deferred move (a coordinated group move)
 * Motor start times are synchronized
 * @param [in] axes - The list of axis/motors in the move (eg. "ABCD")
 * @return motor driver status code.
 */
asynStatus GalilController::beginSyncStartOnlyMove(char *axes)
{
  GalilAxis *pAxis;			//GalilAxis instance
  unsigned i;				//Looping
  asynStatus status;			//Return status

  //Start the group of motors
  status = beginGroupMotion(axes);
  //Loop through the axes list for this move
  //Turn off deferredMove_ for these axis as move has been started
  for (i = 0; axes[i] != '\0'; i++)
	{
	//Retrieve axis specified in axes list
	pAxis = getAxis(axes[i] - AASCII);
	//Process or skip
	if (!pAxis) continue;
	//Set flag
	pAxis->deferredMove_ = false;
	}

  //Return status
  return status;
}

//Sets up position, velocity, and accel for list of motors
//Synchronize motor start times
//Stop times maybe synchronized depending on kinematics
//Kinematics are obeyed for position, velocity and acceleration
asynStatus GalilController::prepSyncStartOnlyMoves(void)
{
   const char *functionName = "prepSyncStartOnlyMoves";
   GalilAxis *pAxis;			//GalilAxis instance
   unsigned axis;			//Axis looping
   char axes[MAX_GALIL_AXES] = {0};	//Constructed list of axis in the deferred move

   //Loop through the axis looking for deferredMoves
   for (axis = 0; axis < MAX_GALIL_AXES; axis++)
      {
      pAxis = getAxis(axis);
      //Skip loop if !pAxis 
      if (!pAxis) continue;
      //Check axis for Sync motor start only deferred move
      if (pAxis->deferredMove_ && !pAxis->deferredMode_)
         {
         //Deferred move found
         //Check motor is good to go
         if (pAxis->beginCheck(functionName, pAxis->deferredVelocity_, false))
            {
            //Motor has problem, so turn off deferred move for this axis
            pAxis->deferredMove_ = false;
            continue;
            }
         //Store axis in list
         sprintf(axes, "%s%c", axes, pAxis->axisName_);
         //Set the acceleration and velocity for this axis
         pAxis->setAccelVelocity(pAxis->deferredAcceleration_, pAxis->deferredVelocity_);
         //Set position
         if (pAxis->deferredRelative_)
            sprintf(cmd_, "PR%c=%.0lf", pAxis->axisName_, pAxis->deferredPosition_);
         else
            sprintf(cmd_, "PA%c=%.0lf", pAxis->axisName_, pAxis->deferredPosition_);
         sync_writeReadController();
         }
      }

   //If at least one axis was found with a deferred move, then start it
   if (strcmp(axes, ""))
      beginSyncStartOnlyMove(axes);

   return asynSuccess;
}

/**
 * Process deferred moves for a controller
 * @return motor driver status code.
 */
asynStatus GalilController::setDeferredMoves(bool deferMoves)
{
  //const char *functionName = "GalilController::setDeferredMoves";
  GalilCSAxis *pCSAxis;		//GalilCSAxis pointer
  unsigned axis;		//Axis looping

  //If we are not ending deferred moves then return
  if (deferMoves || !movesDeferred_)
     {
     movesDeferred_ = true;
     return asynSuccess;
     }

  //We are ending deferred moves.  So process them
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "Processing deferred moves on Galil: %s\n", this->portName);

  //Execute deferred moves
  //Sync start and stop moves
  prepSyncStartStopMoves();
  //Sync start only moves
  prepSyncStartOnlyMoves();

  //Clear deferredMove flag on all CSAxis
  for (axis = MAX_GALIL_AXES; axis < MAX_GALIL_AXES + MAX_GALIL_CSAXES; axis++)
     {
     //Retrieve the CSAxis
     pCSAxis = getCSAxis(axis);
     //Clear deferredMoves flag
     if (pCSAxis)
        pCSAxis->deferredMove_ = false;
     }

  //Deferred moves have been started
  movesDeferred_ = false;

  return asynSuccess;
}

/** Attempts to read value from controller, returns last value set if fails.  
  ** Called by GaLilController::readInt32()
  * \param[in] cmd to send to controller
  * \param[out] value Address of the value to read. 
  * \param[in] axisNo is asyn Param list number 0 - 7.  Controller wide values use list 0 */
asynStatus GalilController::get_integer(int function, epicsInt32 *value, int axisNo = 0)
{
  asynStatus status;				 //Communication status.
	
  if ((status = sync_writeReadController()) == asynSuccess)
     *value = (epicsInt32)atoi(resp_);
  else    //Comms error, return last ParamList value set using setIntegerParam
     getIntegerParam(axisNo, function, value);
  return status;
}

/** Called when asyn clients call pasynInt32->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[out] value Address of the value to read. */
asynStatus GalilController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;		 //function requested
    asynStatus status;				 //Used to work out communication_error_ status.  asynSuccess always returned
    GalilAxis *pAxis = getAxis(pasynUser);	 //Retrieve the axis instance

    //If provided addr does not return an GalilAxis instance, then return asynError
    if (!pAxis) return asynError;

    //We dont retrieve values for records at iocInit.  
    //For output records autosave, or db defaults are pushed to hardware instead
    if (!dbInitialized) return asynError;
    
    if (function == GalilHomeType_)
        {
	//Read home type from controller
        strcpy(cmd_, "MG _CN1");
	status = get_integer(GalilHomeType_, value);
	if (!status)
		*value = (*value > 0) ? 1 : 0;
	}
    else if (function == GalilLimitType_)
	{
	//Read limit type from controller
	strcpy(cmd_, "MG _CN0");
	status = get_integer(GalilLimitType_, value);
	if (!status)
		*value = (*value > 0) ? 1 : 0;
	}
   else if (function == GalilMainEncoder_ || function == GalilAuxEncoder_)
	{
        unsigned setting;
	int main, aux;
	
	sprintf(cmd_ , "CE%c=?", pAxis->axisName_);
	if ((status = sync_writeReadController()) == asynSuccess)
		{
		setting = (unsigned)atoi(resp_);
		//Separate setting into main and aux
		main = setting & 3;
		aux = setting & 12;
		*value = (function == GalilMainEncoder_) ? main : aux;
		}
	else
		{
		//Comms error, return last ParamList value set using setIntegerParam
		if (function == GalilMainEncoder_)
			{
			getIntegerParam(pAxis->axisNo_, GalilMainEncoder_, &main);
			*value = main;
			}
		else
			{
			getIntegerParam(pAxis->axisNo_, GalilAuxEncoder_, &aux);
			*value = aux;
			}
		}
	}
   else if (function == GalilMotorType_)
	{
	float motorType;
	sprintf(cmd_, "MG _MT%c", pAxis->axisName_);
	if ((status = sync_writeReadController()) == asynSuccess)
		{
		motorType = (float)atof(resp_);
		//Upscale by factor 10 to create integer representing motor type 
		*value = (int)(motorType * 10.0);
		//Translate motor type into 0-7 value for mbbi record
		switch (*value)
			{
			case 10:   *value = 0;
				   break;
			case -10:  *value = 1;
				   break;
			case -20:  *value = 2;
				   break;
			case 20:   *value = 3;
				   break;
			case -25:  *value = 4;
				   break;
			case 25:   *value = 5;
				   break;
			case 15:   *value = 6;
				   break;
			case -15:  *value = 7;
				   break;
			default:   break;
			}
		}
	else    //Comms error, return last ParamList value set using setIntegerParam
		getIntegerParam(pAxis->axisNo_, function, value);
	}
  else if (function >= GalilSSIInput_ && function <= GalilSSIData_)
	{
	int ssicapable;	//Local copy of GalilSSICapable_
	//Retrieve GalilSSICapable_ param
	getIntegerParam(GalilSSICapable_, &ssicapable);
	if (ssicapable)
		status = pAxis->get_ssi(function, value);
	else
		status = asynSuccess;
	}
  else if (function == GalilCoordSys_)
	{
	//Read active coordinate system
	sprintf(cmd_, "MG _CA");
	status = get_integer(GalilCoordSys_, value);
	if (!status)
		*value = (*value > 0) ? 1 : 0;
	}
  else if (function == GalilOffOnError_)
	{
	sprintf(cmd_, "MG _OE%c", pAxis->axisName_);
	status = get_integer(GalilOffOnError_, value);
	}
  else if (function == GalilAmpGain_)
	{
	sprintf(cmd_, "MG _AG%c", pAxis->axisName_);
	status = get_integer(GalilAmpGain_, value);
	}
  else if (function == GalilAmpCurrentLoopGain_)
	{
	sprintf(cmd_, "MG _AU%c", pAxis->axisName_);
	status = get_integer(GalilAmpCurrentLoopGain_, value);
	}
  else if (function == GalilAmpLowCurrent_)
	{
	sprintf(cmd_, "MG _LC%c", pAxis->axisName_);
	status = get_integer(GalilAmpLowCurrent_, value);
	}
  else 
	status = asynPortDriver::readInt32(pasynUser, value);

   //Always return success. Dont need more error mesgs
   return asynSuccess;	
}

/** Attempts to read value from controller, returns last good or default if fails.  
  ** Called by GaLilController::readFloat64()
  * \param[in] cmd to send to controller
  * \param[in] asyn Param function number
  * \param[out] value Address of the value to read. 
  * \param[in] axisNo is asyn Param list number 0 - 7.  Controller wide values use list 0 */
asynStatus GalilController::get_double(int function, epicsFloat64 *value, int axisNo = 0)
{
  asynStatus status;				 //Communication status.

  if ((status = sync_writeReadController()) == asynSuccess)
     *value = (epicsFloat64)atof(resp_);
  else    //Comms error, return last ParamList value set using setDoubleParam
     getDoubleParam(axisNo, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the value to read. */
asynStatus GalilController::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
  int function = pasynUser->reason;		//function requested
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  asynStatus status;				//Asyn status
  int addr;					//Address

  //Retrieve address.  Used for analog IO
  status = getAddress(pasynUser, &addr); 
  if (status != asynSuccess) return(status);

  //We dont retrieve values for records at iocInit.  
  //For output records autosave, or db defaults are pushed to hardware instead
  if (!dbInitialized) return asynError;

  if (function == GalilStepSmooth_)
     {
     if (pAxis)
        {
        sprintf(cmd_, "MG _KS%c", pAxis->axisName_);
        get_double(GalilStepSmooth_, value, pAxis->axisNo_);
        }
     }
  else if (function == GalilErrorLimit_)
     {
     if (pAxis)
        {
        sprintf(cmd_, "MG _ER%c", pAxis->axisName_);
        get_double(GalilErrorLimit_, value, pAxis->axisNo_);
        }
     }
  else if (function == GalilUserCmd_)
     {
     //For when input records are set to periodic scan not I/O Intr
     epicsSnprintf(cmd_, sizeof(cmd_), "%s", (const char*)pasynUser->userData);
     get_double(GalilUserVar_, value);
     }
  else if (function == GalilUserVar_)
     {
     //For when input records are set to periodic scan not I/O Intr
     epicsSnprintf(cmd_, sizeof(cmd_), "%s=?", (const char*)pasynUser->userData);
     get_double(GalilUserVar_, value);
     }
  else if (function == GalilAnalogIn_ && !status)
     {
     //Likely a DMC4xxx with low number of axes
     //User wants analog input value > numaxes and so the data is not in datarecord
     //For when input records are set to periodic scan not I/O Intr
     sprintf(cmd_, "MG @AN%d", addr);
     get_double(GalilAnalogIn_, value, addr);
     }
  else  //Command not found
     asynPortDriver::readFloat64(pasynUser, value);

  //Always return success. Dont need more error mesgs
  return asynSuccess;	
}

/** Called when asyn clients call pasynUInt32Digital->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write.
  * \param[in] mask Mask value to use when writinging the value. */
asynStatus GalilController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    int function = pasynUser->reason;
    int addr=0;
    asynStatus status = asynSuccess;
    const char* functionName = "writeUInt32Digital";
    int obwpa;			//How many output bits are supported per addr at the record layer for this controller type
    epicsUInt32 maxmask;	//Maximum binary out mask supported at record layer for this controller
    int i;

    //Retrieve record address.  Which is byte, or word here.
    status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setUIntDigitalParam(addr, function, value, mask);

    if (function == GalilBinaryOut_)
  	{
	//Ensure record mask is within range
	maxmask = (rio_) ? 0x80 : 0x8000;

	if (mask > maxmask && strcmp("Unknown", model_))
		{
		printf("%s model %s mask too high @ > %x addr %d mask %x\n", functionName, model_, maxmask, addr, mask);
		return asynSuccess;
		}
		
	//Determine bit i from mask
	for (i=0;i<16;i++)
		{
		if (pow(2.0,i) == mask)
			{
			//Bit numbering is different on DMC compared to RIO controllers
			if (!rio_)
				{
				obwpa = 16;	//Binary out records for DMC support 16 bits per addr
				i++;		//First bit on motor controllers is bit 1
				}
			else
				obwpa = 8;	//Binary out records for RIO support 8 bits per addr
			//Set or clear bit as required
			if (value == mask)
				sprintf(cmd_, "SB %d", (addr * obwpa) + i);
			else
				sprintf(cmd_, "CB %d", (addr * obwpa) + i);

			//Write setting to controller
			sync_writeReadController();
			//We found the correct bit, so break from loop
			break;
			}
		}
	}

    //Always return success. Dont need more error mesgs
    return asynSuccess;
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus GalilController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;		//Function requested
  int addr=0;				        //Address requested
  asynStatus status;				//Used to work out communication_error_ status.  asynSuccess always returned
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  GalilCSAxis *pCSAxis = getCSAxis(pasynUser);	//Retrieve the axis instance
  int hometype, limittype;			//The home, and limit switch type
  int mainencoder, auxencoder, encoder_setting; //Main, aux encoder setting
  char coordinate_system;			//Coordinate system S or T
  char axes[MAX_GALIL_AXES];			//Coordinate system axis list
  double eres, mres;				//mr eres, and mres
  float oldmotor;				//Motor type before changing it.  Use Galil numbering
  unsigned i;					//Looping
  float oldmtr_abs, newmtr_abs;			//Track motor changes
  int uploading;				//Array uploading status

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return(status);

  //Check axis instance
  if (addr < MAX_GALIL_AXES && !rio_)
     {
     if (!pAxis) return asynError;
     }
  else if (!rio_)
     {
     if (!pCSAxis) return asynError;
     }

  //Protect against calling upload array whilst status is uploading
  getIntegerParam(GalilUserArrayUpload_, &uploading);
  if (uploading && function == GalilUserArrayUpload_)//User requesting upload, but its still running
     return asynSuccess;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(addr, function, value);
  
  if (function == GalilHomeType_ || function == GalilLimitType_)
  	{
	//Retrieve the limit type and home type
	//Must ensure getIntegerParam successful, as some parameters may not be set yet due to record default mechanism
	if ((getIntegerParam(GalilLimitType_, &limittype) == asynSuccess) && (getIntegerParam(GalilHomeType_, &hometype) == asynSuccess))
		{
		//Convert Param syntax of these params to controller syntax
		limittype = (limittype > 0) ? 1 : -1;
		hometype = (hometype > 0) ? 1 : -1;
		//Assemble cmd string 
		sprintf(cmd_, "CN %d,%d,-1,0,1", limittype, hometype);
		//printf("GalilLimitType cmd:%s\n", cmd_);
		//Write setting to controller
		status = sync_writeReadController();
		}
  	}
  else if (function == GalilAuxEncoder_ || function == GalilMainEncoder_)	
	{
	//Retrieve main and aux encoder setting
	//Must ensure getIntegerParam successful, as some parameters may not be set yet due to record default mechanism
	if ((getIntegerParam(pAxis->axisNo_, GalilMainEncoder_, &mainencoder) == asynSuccess) && (getIntegerParam(pAxis->axisNo_, GalilAuxEncoder_, &auxencoder) == asynSuccess))
		{
		//Assemble cmd string 
		encoder_setting = mainencoder + auxencoder;
		sprintf(cmd_, "CE%c=%d", pAxis->axisName_, encoder_setting);
		//printf("GalilMainEncoder cmd:%s value=%d\n", cmd_, value);
		//Write setting to controller
		status = sync_writeReadController();
		}
	}
  else if (function == GalilBrake_)
	{
	if (value)
		{
		status = pAxis->setBrake(true);
		pAxis->brakeInit_ = true;
		}
	else
		{
		status = pAxis->setBrake(false);
		pAxis->brakeInit_ = false;
		}
	//printf("GalilBrake_ cmd:%s value=%d\n", cmd_, value);
	}
  else if (function == GalilMotorType_)
	{
	float newmtr;

	//Query motor type before changing setting
	sprintf(cmd_, "MT%c=?", pAxis->axisName_);
	sync_writeReadController();
	oldmotor = (float)atof(resp_);

	//Assemble command to change motor type
	switch (value)
		{
		case 0: newmtr = 1.0;
			break;
		case 1: newmtr = -1.0;
			break;
		case 2: newmtr = -2.0;
			break;
		case 3: newmtr = 2.0;
			break;
		case 4: newmtr = -2.5;
			break;
		case 5: newmtr = 2.5;
			break;
		case 6: newmtr = 1.5;
			break;
		case 7: newmtr = -1.5;
			break;
		default: newmtr = 1.0;
			break;
		}

	//Change motor type
	sprintf(cmd_, "MT%c=%1.1f", pAxis->axisName_, newmtr);
	//printf("GalilMotorType_ cmd:%s value %d\n", cmd_, value);
	//Write setting to controller
	status = sync_writeReadController();

	//Determine if controller will use main or auxillary register with selected motor type
	pAxis->ctrlUseMain_ = (value < 2 || value > 5) ? true : false;

	//IF motor was servo, and now stepper
	//Galil hardware MAY push main encoder to aux encoder (stepper count reg)
	//We re-do this, but apply encoder/motor scaling
	if (fabs(oldmotor) <= 1.5 && value > 1 && value < 6)
		{
		getDoubleParam(pAxis->axisNo_, GalilEncoderResolution_, &eres);
		getDoubleParam(pAxis->axisNo_, motorResolution_, &mres);
		if (mres != 0.000000)
			{
			//Calculate step count from existing encoder_position, construct mesg to controller_
			sprintf(cmd_, "DP%c=%.0lf", pAxis->axisName_, pAxis->encoder_position_ * (eres/mres));
			//Write setting to controller
			status = sync_writeReadController();
			}
		}

	//IF motor was stepper, and now servo
	//Set reference position equal to main encoder, which sets initial error to 0
	if (fabs(oldmotor) > 1.5 && (value < 2 || value > 5))
		{
		//Calculate step count from existing encoder_position, construct mesg to controller_
		sprintf(cmd_, "DP%c=%.0lf", pAxis->axisName_, pAxis->encoder_position_);
		//Write setting to controller
		status = sync_writeReadController();
		}

	//Changing motor polarity will change motor limits direction consistency
	oldmtr_abs = abs(oldmotor);
	newmtr_abs = abs(newmtr);
	if (((oldmtr_abs == newmtr_abs) && ((oldmotor > 0 && newmtr < 0) || (oldmotor < 0 && newmtr > 0))) ||
		((oldmtr_abs == 2.0 && newmtr_abs == 2.5) || (oldmtr_abs == 2.5 && newmtr_abs == 2.0)) ||
		((oldmotor == 1.0 && newmtr == -1.5) || (oldmtr_abs == -1.5 && newmtr_abs == 1.0)) ||
		((oldmotor == -1.0 && newmtr == 1.5) || (oldmtr_abs == 1.5 && newmtr_abs == -1.0)))
		{
		if (pAxis->limitsDirState_ != unknown)
			pAxis->limitsDirState_ = (pAxis->limitsDirState_ == not_consistent) ? consistent : not_consistent;
		//Ensure WLP Active state is unlatch so a move can be initiated
		if (pAxis->limitsDirState_ == consistent)
			setIntegerParam(pAxis->axisNo_, GalilWrongLimitProtectionActive_, 0);
		}
	else if (oldmotor != newmtr)
		pAxis->limitsDirState_ = unknown;
	}
  else if (function == GalilUseEncoder_)
	{
	//This is one of the last items pushed into driver at startup so flag
	//Axis now ready for move commands
	if (addr < MAX_GALIL_AXES)
		{
		pAxis->axisReady_ = true;//Real motor
		//Restore brake cmd state
		pAxis->restoreBrake();
		}
	else
		pCSAxis->axisReady_ = true;//CS motor
	}
  else if (function >= GalilSSIInput_ && function <= GalilSSIData_)
	{
	int ssicapable;	//Local copy of GalilSSICapable_
	//Retrieve GalilSSICapable_ param
	getIntegerParam(GalilSSICapable_, &ssicapable);
	
	//Only if controller is SSI capable
	if (ssicapable)
		status = pAxis->set_ssi();
	}
  else if (function == GalilSSIInvert_)
	{
	pAxis->invert_ssi_ = (value == 0) ? false : true;
        }
  else if (function == GalilOffOnError_)
	{
	sprintf(cmd_, "OE%c=%d", pAxis->axisName_, value);
	//printf("GalilOffOnError_ cmd:%s value %d\n", cmd, value);
	//Write setting to controller
	status = sync_writeReadController();
	}
  else if (function == GalilCoordSys_)
	{
	coordinate_system = (value == 0) ? 'S' : 'T';
	sprintf(cmd_, "CA %c", coordinate_system);
	//printf("GalilCoordSys_ cmd:%s value %d\n", cmd, value);
	//Write setting to controller
	status = sync_writeReadController();
	}
  else if (function == GalilCoordSysMotorsStop_ || function == GalilCoordSysMotorsGo_)
	{
	//Decide Stop or Go function
	int motor_spmg = (function == GalilCoordSysMotorsStop_) ? 0 : 3;
	//If Stop function, then stop coordsys
        if (function == GalilCoordSysMotorsStop_)
		{
		sprintf(cmd_, "ST %c", (addr == 0) ? 'S' : 'T');
		//Write setting to controller
		status = sync_writeReadController();
		}
	//Retrieve coordSys axes list
        getStringParam(addr, GalilCoordSysMotors_, MAX_GALIL_AXES, axes);
	//Stop/Go all motors in the list
	//This is done to stop motor backlash correction after coordsys stop
	for (i = 0; axes[i] != '\0'; i++)
		{
		//Stop/go the motor
		setIntegerParam(axes[i] - AASCII, GalilMotorStopGo_, motor_spmg);
		}
	callParamCallbacks();
	}
  else if (function == GalilOutputCompareAxis_)
	{
	status = setOutputCompare(addr);
	}
  else if (function == GalilUserArrayUpload_)
	{
	epicsEventSignal(arrayUploadEvent_);
	}
  else if (function == GalilAmpGain_)
	{
	int motorOff = 0;
	//Retrieve motor off status
	sprintf(cmd_, "MG _MO%c", pAxis->axisName_);
	status = sync_writeReadController();
	if (!status)
		motorOff = atoi(resp_);
	if (!motorOff)
		{
		//Motor must be off to change gain
		sprintf(cmd_, "MO%c", pAxis->axisName_);
		sync_writeReadController();
		}
	//Motor is now off, we can change gain
	sprintf(cmd_, "AG%c=%d", pAxis->axisName_, value);
	sync_writeReadController();
	if (!motorOff)
		{
		//Motor was on at start, so turn it back on
		sprintf(cmd_, "SH%c", pAxis->axisName_);
		sync_writeReadController();
		}
	}
  else if (function == GalilAmpCurrentLoopGain_)
	{
	float gainSetting = 0;
	//Find the correct gain setting
	if (value == 1)
		gainSetting = 0.5;
	else if (value == 2)
		gainSetting = 1;
	else if (value == 3)
		gainSetting = 1.5;
	else if (value == 4)
		gainSetting = 2;
	else if (value == 5)
		gainSetting = 3;
	else if (value == 6)
		gainSetting = 4;
	//Set current loop gain
	sprintf(cmd_, "AU%c=%.1f", pAxis->axisName_, gainSetting);
	sync_writeReadController();
	}
  else if (function == GalilAmpLowCurrent_)
	{
	//Set low current mode
	sprintf(cmd_, "LC%c=%d", pAxis->axisName_, value);
	sync_writeReadController();
	}
  else 
	{
	/* Call base class method */
	status = asynMotorController::writeInt32(pasynUser, value);
  	}

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus GalilController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;		//Function requested
  asynStatus status;				//Used to work out communication_error_ status.  asynSuccess always returned
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  int addr=0;					//Address requested

  //Retrieve address.  Used for analog IO
  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return(status);

  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(addr, function, value);
     
  if (function == GalilStepSmooth_)
     {
     if (pAxis)
        {
        //Write new stepper smoothing factor to GalilController
        sprintf(cmd_, "KS%c=%lf",pAxis->axisName_, value);
        //printf("GalilStepSmooth_ cmd:%s value %lf\n", cmd, value);
        status = sync_writeReadController();
        }
     }
  else if (function == GalilErrorLimit_)
     {
     if (pAxis)
        {
        //Write new error limit to GalilController
        sprintf(cmd_, "ER%c=%lf",pAxis->axisName_, value);
        //printf("GalilErrorLimit_ cmd:%s value %lf\n", cmd, value);
        status = sync_writeReadController();
        }
     }
  else if (function == GalilUserDataDeadb_)
     {
     if (pAxis)//Store user data deadband value for use in GalilPoller
        pAxis->userDataDeadb_ = value;
     }
  else if (function == GalilAnalogInDeadb_)
     {
     if (addr >= 0 && addr <= ANALOG_PORTS)
        analogIndeadb_[addr] = value;
     }
  else if (function == GalilAnalogOutRBVDeadb_)
     {
     if (addr >= 0 && addr <= ANALOG_PORTS)
        analogOutRBVdeadb_[addr] = value;
     }
  else if (function == GalilAnalogOut_)
     {
     //Write new analog value to specified output (addr)
     sprintf(cmd_, "AO %d, %f", addr, value);
     //printf("GalilAnalogOut_ cmd:%s value %lf\n", cmd, value);
     status = sync_writeReadController();
     }
  else if (function == GalilOutputCompareStart_ || function == GalilOutputCompareIncr_)
     {
     status = setOutputCompare(addr);
     }
  else if (function == GalilUserCmd_)
     {
     epicsSnprintf(cmd_, sizeof(cmd_), "%s", (const char*)pasynUser->userData);
     if ( (status = sync_writeReadController()) == asynSuccess)
        setDoubleParam(0, function, atof(resp_));  //For when input records set to I/O Intr
     }
  else if (function == GalilUserVar_)
     {
     //Store user variables for use in unsolicited messaging
     if (find(userVariables_.begin(), userVariables_.end(), (const char*)pasynUser->userData) == userVariables_.end())
        {
        //This user variable has not been stored yet, so store it now
        userVariables_.push_back((const char*)pasynUser->userData);
        userVariableAddresses_.push_back(addr);
        }
     //Set user variable value on controller
     epicsSnprintf(cmd_, sizeof(cmd_), "%s=%lf", (const char*)pasynUser->userData, value);
     status = sync_writeReadController();
     }
  else
     {
     /* Call base class method */
     status = asynMotorController::writeFloat64(pasynUser, value);
     }

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

//  Overriden from asynMotorController as we need to support CSAxis profiles
/** Called when asyn clients call pasynFloat64Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus GalilController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
  int function = pasynUser->reason;		//Reason this method was called
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  GalilCSAxis *pCSAxis = getCSAxis(pasynUser);	//Retrieve the CSAxis instance
  asynStatus status;				//Status
  int addr;					//address

  //Retrieve address of caller
  status = getAddress(pasynUser, &addr); 
  if (status != asynSuccess) return(status);

  //Check axis instance the easy way since no RIO commands in writeFloat64Array
  if (addr < MAX_GALIL_AXES)
     {
     if (!pAxis) return asynError;
     }
  else
     {
     if (!pCSAxis) return asynError;
     }
  
  if (nElements > maxProfilePoints_) nElements = maxProfilePoints_;
   
  if (function == profileTimeArray_) {
    memcpy(profileTimes_, value, nElements*sizeof(double));
  } 
  else if (function == profilePositions_) {
    if (addr < MAX_GALIL_AXES)
       pAxis->defineProfile(value, nElements);
    else
       pCSAxis->defineProfile(value, nElements);
  }
  else  //Call parent class method 
     asynMotorController::writeFloat64Array(pasynUser, value, nElements);
  return asynSuccess;
}

asynStatus GalilController::writeOctet(asynUser *pasynUser, const char*  value,  size_t  nChars,  size_t *  nActual)
{
  int function = pasynUser->reason;		//Function requested
  int status = asynSuccess;			//Used to work out communication_error_ status.  asynSuccess always returned
  double aivalue;				//Convert response to value
  unsigned i;					//looping
  char mesg[MAX_GALIL_STRING_SIZE];		//Controller mesg
  GalilCSAxis *pCSAxis;				//Pointer to CSAxis instance
  int addr=0;					//Address requested

  //Retrieve address
  if (getAddress(pasynUser, &addr))
     return asynError;

  std::string value_s(value, nChars);  // in case value is not NULL terminated

  //Set value in paramlist
  setStringParam(addr, function, value_s.c_str());
  //Num of chars written to paramList
  *nActual = nChars;

  /* Set the parameter and readback in the parameter library. */
  if (function == GalilUserOctet_)
     {
     //Send the command	
     epicsSnprintf(cmd_, sizeof(cmd_), "%s", value_s.c_str());
     if ( (status = sync_writeReadController()) == asynSuccess )
        {
        //Set readback value(s) = response from controller
        //String monitor
        setStringParam(GalilUserOctet_, resp_);
        //ai monitor
        aivalue = atof(resp_);
        setDoubleParam(0, GalilUserOctetVal_, aivalue);
        }
     else
        {
        //Set readback value = response from controller
        setStringParam(GalilUserOctet_, "error");
        }
     }
  else if (function >= GalilCSMotorForward_ && function <= GalilCSMotorReverseH_)
     {
     //User has entered a new kinematic transform equation
     //We MUST loop through all the cs axis
     for (i = MAX_GALIL_AXES; i < MAX_GALIL_CSAXES + MAX_GALIL_AXES; i++)
        {
        //Retrieve the cs axis instance
        pCSAxis = getCSAxis(i);
        if (!pCSAxis) continue;
        //Parse the transforms and place results in GalilCSAxis instance(s)
        status |= pCSAxis->parseTransforms();
        //Flag kinematics have been altered
        if (!status)
           pCSAxis->kinematicsAltered_ = true;
        }
     //Tell user when success
     //Dont provide message during startup
     if (pCSAxis)
        if (!status && pCSAxis->axisReady_)
           {
           sprintf(mesg, "Kinematics changed successfully");
           setCtrlError(mesg);
           }
     }
  else
     {
     /* Call base class method */
     status = asynMotorController::writeOctet(pasynUser, value,  nChars, nActual);
     }

  //Update params
  callParamCallbacks();

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

//Process unsolicited message from the controller
//Called by poll thread.  Must not lock, or use writeReadController
void GalilController::processUnsolicitedMesgs(void)
{
   char *charstr;		//The current token
   GalilAxis *pAxis;	 	//GalilAxis
   char rawbuf[MAX_GALIL_STRING_SIZE * MAX_GALIL_AXES];//Unsolicited message(s) buffer
   char mesg[MAX_GALIL_STRING_SIZE];	//An individual message
   char axisName;		//Axis number message is for
   int value;			//The value contained in the message
   double valuef;		//The value contained in the message
   int addr;			//Address of user defined record
   vector<string>::iterator it;	//Iterator where user variable found
   ptrdiff_t index;		//Index where user variable found
   char *tokSave = NULL;	//Remaining tokens
   int uploading;		//Array uploading status
   bool found;			//Found a user variable message
   int len;			//length of received message

   //Collect unsolicited message from controller
   len = unsolicitedQueue_.tryReceive(rawbuf, MAX_GALIL_STRING_SIZE);

   //Process the message
   if (len > 0)
      {
      //Terminate the buffer
      rawbuf[len] = '\0';
      //Take backup before splitting into tokens
      string rawbufOriginal = rawbuf;
      //Break message into tokens: name value name value    etc.
      charstr = epicsStrtok_r(rawbuf, " \r\n", &tokSave);
      while (charstr != NULL)
         {
         //Extract the message
         strncpy(mesg, charstr, strlen(charstr));
         //Null terminate the message
         mesg[strlen(charstr)] = '\0';

         //Process known messages
         if (!abs(strcmp(mesg, "Upload")))
            {
            //Array upload message
            //Protect against calling upload array whilst status is uploading
            getIntegerParam(GalilUserArrayUpload_, &uploading);
            if (!uploading)
               epicsEventSignal(arrayUploadEvent_); //Wake up array upload thread
            }
         else if (!abs(strncmp(mesg, "home", 4)))
            {
            //Home related message
            //Determine axis message is for
            axisName = (char)mesg[strlen(mesg)-1];
            //Extract and Null terminate the message
            mesg[strlen(mesg)-1] = '\0';
            //Retrieve GalilAxis instance for the correct axis
            pAxis = getAxis(axisName - AASCII);
            //Retrieve the value
            charstr = epicsStrtok_r(NULL, " \r\n", &tokSave);
            if (charstr != NULL && pAxis)
               {
               value = atoi(charstr);
               //Process known messages
               //Motor homed message
               if (!abs(strcmp(mesg, "homed")))
                  {
                  //Send homed message to pollServices only if homed%c=1
                  if (value)
                     {
                     //Send homed message to pollServices
                     pAxis->homedExecuted_ = false;
                     pAxis->pollRequest_.send((void*)&MOTOR_HOMED, sizeof(int));
                     pAxis->homedSent_ = true;
                     }
                  else
                     {
                     //Maintain homing asynParam that includes JAH
                     //Homed failed, so dont do JAH
                     setIntegerParam(pAxis->axisNo_, GalilHoming_, 0);
                     }
                  //Set homing flag false
                  //This homing flag does not include JAH
                  pAxis->homing_ = false;
                  //Set motorRecord MSTA bit 15 motorStatusHomed_
                  //Homed is not part of Galil data record, we support it using Galil code and unsolicited messages instead
                  //We must use asynMotorAxis version of setIntegerParam to set MSTA bits for this MotorAxis
                  pAxis->setIntegerParam(motorStatusHomed_, value);
                  callParamCallbacks();
                  }
               }
            }//Home related messages
         else
            {
            //Unknown or User variable message
            //Default user variable message flag
            found = false;
            //Retrieve the value
            charstr = epicsStrtok_r(NULL, " \r\n", &tokSave);
            if (charstr != NULL  && userVariables_.size())
               {
               //Attempt to find the user variable
               it = find(userVariables_.begin(), userVariables_.end(), mesg);
               if (it != userVariables_.end())
                  {
                  //User variable found
                  found = true;
                  //Calculate exact index of user variable
                  index = it - userVariables_.begin();
                  //Grab the user defined record address
                  addr = userVariableAddresses_[index];
                  //Extract the new value for user define record
                  valuef = atof(charstr);
                  //Update the record with the new value
                  setDoubleParam(addr, GalilUserVar_, valuef);
                  callParamCallbacks(addr);
                  }
               }
            //Check for unknown message type
            if (!found && userVariables_.size())
               {
               //Unknown message
               //Display message in controller console
               rawbufOriginal.erase(rawbufOriginal.find_last_not_of(" \n\r\t:")+1);
               setCtrlError(rawbufOriginal.c_str());
               callParamCallbacks();
               //All complete
               break;
               }
            }//User variable messages
         //Retrieve next mesg
         charstr = epicsStrtok_r(NULL, " \r\n", &tokSave);
         }//while
      }
}

//Extract controller data from GalilController data record
//Return status of GalilController data record acquisition
void GalilController::getStatus(void)
{
   char src[MAX_GALIL_STRING_SIZE]="\0";	//data source to retrieve
   int addr;					//addr or byte of IO
   int start, end;				//start, and end of analog numbering for this controller
   double paramDouble;				//For passing asynFloat64 to ParamList
   unsigned paramUDig;				//For passing UInt32Digital to ParamList
   unsigned in = 0, out = 0;			//For passing digital in/out for DMC30000 only
   int i;					//Looping
  
   //If data record query success in GalilController::acquireDataRecord
   if (recstatus_ == asynSuccess && connected_)
	{
	//extract relevant controller data from GalilController record, store in GalilController
	//If connected, then proceed

	//DMC30000 series only.
	if (model_[3] == '3')
		{
		//First 8 input, and first 4 output bits only
		for (i = 1; i <= 8; i++)
			{
			//Digital input bit
			strcpy(src, "@IN[x]");
			src[4] = i + 48;
			paramUDig = (unsigned)sourceValue(recdata_, src);
			in += paramUDig << (i - 1);
			//Digital output bits
			if (i <= 4)
				{
				//Database records are arranged by word
				//ValueMask = 0xFFFF because a word is 16 bits
				strcpy(src, "@OUT[x]");
				src[5] = i + 48;
				paramUDig = (unsigned)sourceValue(recdata_, src);
				out += paramUDig << (i - 1);
				}
			}
		//ValueMask = 0xFF because a byte is 8 bits
		//Database records are arranged by byte
		//Callbacks happen on value change
		setUIntDigitalParam(0, GalilBinaryIn_, in, 0xFF );
		//Database records are arranged by word
		//ValueMask = 0xFFFF because a word is 16 bits
		setUIntDigitalParam(0, GalilBinaryOutRBV_, out, 0xFFFF );
		}
	else
		{
		//for all models except DMC30000 series
		//digital inputs in banks of 8 bits for all models except DMC30000 series
		for (addr=0;addr<BINARYIN_BYTES;addr++)
			{
			strcpy(src, "_TIx");
			src[3] = addr + 48;
			paramUDig = (unsigned)sourceValue(recdata_, src);
			//ValueMask = 0xFF because a byte is 8 bits
			//Callbacks happen on value change
			setUIntDigitalParam(addr, GalilBinaryIn_, paramUDig, 0xFF );
			//Example showing forced callbacks even if no value change
			//setUIntDigitalParam(addr, GalilBinaryIn_, paramUDig, 0xFF, 0xFF );
			}
		//data record has digital outputs in banks of 16 bits for dmc, 8 bits for rio
		for (addr=0;addr<BINARYOUT_WORDS;addr++)
			{
			strcpy(src, "_OPx");
			src[3] = addr + 48;
			paramUDig = (unsigned)sourceValue(recdata_, src);
			//ValueMask = 0xFFFF because a word is 16 bits
			//Callbacks happen on value change
			setUIntDigitalParam(addr, GalilBinaryOutRBV_, paramUDig, 0xFFFF );
			//Example showing forced callbacks even if no value change
			//setUIntDigitalParam(addr, GalilBinaryOutRBV_, paramUDig, 0xFFFF, 0xFFFF );
			}
		}
	//Analog ports
	//Port numbering is different on DMC compared to RIO controllers
	start = (rio_) ? 0 : 1;
        end = ANALOG_PORTS + start;

	for (addr = start;addr < end;addr++)
		{
		//Analog inputs
		strcpy(src, "@AN[x]");
		src[4] = addr + 48;
		paramDouble = (double)sourceValue(recdata_, src);
		if ((paramDouble < (analogInPosted_[addr] - analogIndeadb_[addr])) || (paramDouble > (analogInPosted_[addr] + analogIndeadb_[addr])))
			{
			setDoubleParam(addr, GalilAnalogIn_, paramDouble);
			analogInPosted_[addr] = paramDouble;
			}
		if (rio_)
			{
			//Analog output readbacks for rio
			strcpy(src, "@AO[x]");
			src[4] = addr + 48;
			paramDouble = (double)sourceValue(recdata_, src);
			if ((paramDouble < (analogOutRbvPosted_[addr] - analogOutRBVdeadb_[addr])) || (paramDouble > (analogOutRbvPosted_[addr] + analogOutRBVdeadb_[addr])))
				{
				setDoubleParam(addr, GalilAnalogOutRBV_, paramDouble);
				analogOutRbvPosted_[addr] = paramDouble;
				}
			}
		}

	//Process unsolicited mesgs from controller
	processUnsolicitedMesgs();

	//Coordinate system status
	for (addr=0;addr<COORDINATE_SYSTEMS;addr++)
		{
		//Move/done status
		strcpy(src, "_BGx");
		src[3] = (addr) ? 'T' : 'S';
		setIntegerParam(addr, GalilCoordSysMoving_, (int)sourceValue(recdata_, src));

		//Segment count
		strcpy(src, "_CSx");
		src[3] = (addr) ? 'T' : 'S';
		setIntegerParam(addr, GalilCoordSysSegments_, (int)sourceValue(recdata_, src));
		}
	}
}

//Acquire a data record from controller, store in GalilController instance
//Called by GalilPoller::run
asynStatus GalilController::poller(void)
{
	//Acquire a data record
	acquireDataRecord();

	//Extract controller data from data record, store in GalilController, and ParamList
	getStatus();

	//Return value is not monitored by asynMotorController
	return asynSuccess;
}

//Send unsolicited message to queue
asynStatus GalilController::sendUnsolicitedMessage(char *mesg)
{
  //Remove any unwanted characters
  string message = mesg;
  message.erase (std::remove(message.begin(), message.end(), ':'), message.end());
  strcpy(mesg, message.c_str());
  //Send the unsolicited message to the queue
  unsolicitedQueue_.trySend((void *)mesg, (unsigned)strlen(mesg));
  //empty mesg buffer
  mesg[0] = '\0';
  //Success
  return asynSuccess;
}

//Below function supplied for Cygwin, MingGw
bool GalilController::my_isascii(int c)
{
   if (c == 10 || c == 13 || (c >= 48 && c <= 57) || (c >= 65 && c <= 90) ||
       (c >= 97 && c <= 122) || c == 32 || c == 46)
      return true;
   else
      return false;
}

/** Reads a binary data record from the controller
  * pasynUser [in] - pasynUser for the synchronous or asynchronous connection 
  * input [out] - The caller supplied buffer to put data into
  * bytesize [in] - The data record size expected
  */
asynStatus GalilController::readDataRecord(char *input, unsigned bytesize)
{
  asynStatus status;	//Asyn status
  size_t nread = 0;	//Asyn read bytes
  int eomReason;	//Asyn end of message reason
  char buf[MAX_GALIL_DATAREC_SIZE];//Temporary buffer to hold data record in some circumstances
  char mesg[MAX_GALIL_STRING_SIZE] = {0x0};//Unsolicited mesg buffer
  unsigned check;	//Check record size upon receipt
  char previous = 0;	//Byte read in previous cycle
  unsigned char value;	//Used for type conversion
  unsigned i = 0;	//Looping
  unsigned j = 0;	//Unsolicited byte counter
  unsigned k = HEADER_BYTES;//Record byte counter
  bool recstart = false;//Header found
  unsigned readsize = bytesize;//Requested number of bytes may vary in tcp sync mode when unsolicited mesg

  for (;;)
     {
     //Read bytesize using octet interface and user supplied buffer
     if (async_records_)
        status = pAsyncOctet_->read(pAsyncOctetPvt_, pasynUserAsyncGalil_, input, readsize, &nread, &eomReason);
     else
        status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, input, readsize, &nread, &eomReason);

     //Serial mode characters arrive with nread = 1
     //UDP async mode unsolicited mesg always cause read (above) to return with nread set to mesg length
     //TCP sync mode unsolicited mesg sometimes cause read to return with nread set to mesg length
     //TCP sync mode unsolicited mesg mostly cause read to return with nread = readsize
     //Readsize varies when reading data record tail in tcp sync with unsolicited message

     if (nread == bytesize && eomReason == ASYN_EOM_CNT)
        {
        //Read returned ok, with expected bytes
        //Look for record header at expected location in buffer
        //In both tcp sync, and udp async mode, when no unsolicited message
        //Record header can be found here
        //Record header cannot be found here in rs232 mode
        check = (unsigned char)input[3] << 8;
        check = (unsigned char)input[2] + check;
        if (check == datarecsize_)
           return status;//Found record at expected location, job done
        }

     if (!status && nread > 0)
        {
        //Read returned ok, either record header not found, or less than bytesize bytes returned
        //Scan through bytes read looking for record header, unsolicited mesg, and data record body/tail
        for (i = 0; i < nread; i++)
           {
           if (!recstart)
              {
              //Data record has not started
              //Check for record header
              //Calculate record size using this byte, and previous
              //Set previous byte
              if (i != 0)
                 previous = input[i - 1];
              check = (unsigned char)input[i] << 8;
              check = (unsigned char)previous + check;
              //Compare calculated size against datarecsize we are searching for
              if (check == datarecsize_)
                 {
                 //Detected record header
                 recstart = true;
                 if (nread == bytesize && eomReason == ASYN_EOM_CNT)
                    {
                    //Received expected number of bytes, but didn't find header at expected location
                    //Not a serial connection, so it must be synchronous tcp
                    //This means the header is not at expected location due to unsolicited message
                    //Store buffer index where datarecord header starts
                    //Which is also number of data record bytes remaining (tail) to read
                    readsize = i - HEADER_BYTES + 1;
                    }
                 }
              if (!recstart)
                 {
                 //Extract unsolictied message
                 value = (unsigned char)(input[i] - 0x80);
                 if (((input[i] & 0x80) == 0x80) && (my_isascii((int)value)))
                    {
                    //Byte looks like an unsolicited packet
                    //Check for overrun
                    if (j > MAX_GALIL_DATAREC_SIZE - 2)
                       return asynError;//No unsolicited message should be this long return error
                    //Copy potential unsolicited byte into mesg buffer
                    mesg[j++] = (unsigned char)value;
                    mesg[j] = '\0';
                    //Send unsolicited message if last char was line feed
                    if (mesg[j - 1] == '\n')
                       {
                       sendUnsolicitedMessage(mesg);
                       mesg[0] = '\0';
                       j = 0;
                       }
                    }
                 }
              }
           else
              {
              //Data record receipt has started
              //Data record received here in tcp synchronous mode when unsolicited mesg
              //Data record received here in serial mode
              //Copy data record into temp buffer
              //Check for overrun
              if (k > MAX_GALIL_DATAREC_SIZE - 2)
                 return asynError;//Datarecord should not be this long return error
              buf[k++] = input[i];
              //Terminate the buffer
              buf[k]='\0';
              //Reached expected bytes?
              if (k == bytesize)
                 {
                 //Data record read complete
                 //Copy data record into user supplied buffer
                 memcpy(input, buf, bytesize);
                 return status;
                 }
              }
           }

        //Store last byte received this read for next cycle
        previous = input[nread - 1];
        //Loop back and keep reading until we get the data record or error
        }
     else
        return asynError;//Stop if any asyn error
     }
}

//Acquire data record from controller
void GalilController::acquireDataRecord(void)
{
  //const char *functionName="acquireDataRecord";
  size_t nwrite = 0;		//Asyn written bytes
  //epicsTimeStamp endt_;	//Used for debugging, and tracking overall performance
  //epicsTimeStamp startt_;	//Used for debugging, and tracking overall performance
  //double time_taken;		//Used for debugging, and tracking overall performance

  if (connected_)
     {
     //Get acquisition start time
     //epicsTimeGetCurrent(&startt_);
     if (!async_records_)
        { 
        //Synchronous poll
        //Need the lock for synchronous poll
        lock();
        //Prepare QR command
        strcpy(cmd_, "QR\r");
        //Write the QR query to controller
        recstatus_ = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, cmd_, 3, &nwrite);
        if (!recstatus_) //Solicited data record includes an extra colon at the end
           recstatus_ = readDataRecord(resp_, datarecsize_ + 1); //Get the record
        unlock();
        }
     else //Asynchronous poll
        recstatus_ = readDataRecord(asyncresp_, datarecsize_); //Get the record

     //Get acquisition end time
     //epicsTimeGetCurrent(&endt_);
     //Calculate acquistion time
     //time_taken = epicsTimeDiffInSeconds(&endt_, &startt_);
     //if (time_taken > 0.01)
     //printf("%s GalilController::acquire %2.6lfs stat %d\n", model_, time_taken, recstatus_);
     }

  //Track timeouts
  if (recstatus_ != asynSuccess)
     consecutive_timeouts_++;

  //Force disconnect if any errors
  if (consecutive_timeouts_ > ALLOWED_TIMEOUTS)
     disconnect();

  //If no errors, copy the data
  if (!recstatus_ && connected_)
     {
     //No errors
     consecutive_timeouts_ = 0;
     //Copy the returned data record into GalilController
     if (!async_records_)
        recdata_.assign(resp_, resp_ + datarecsize_);
     else
        recdata_.assign(asyncresp_, asyncresp_ + datarecsize_);
     }
}

/** Writes a string to the GalilController controller and reads the response using synchronous communications
  * Calls sync_writeReadController() with default locations of the input and output strings
  * and GalilController timeout. */ 
asynStatus GalilController::sync_writeReadController(bool testQuery)
{
  const char *functionName="sync_writeReadController";
  size_t nread;
  int status;
  size_t len;
  static const char* debug_file_name = macEnvExpand("$(GALIL_DEBUG_FILE=)");
  static FILE* debug_file = ( (debug_file_name != NULL && strlen(debug_file_name) > 0) ? fopen(debug_file_name, "at") : NULL);

  //Simply return asynSuccess if not connected
  //Asyn module corrupts ram if we try write/read with no connection
  if (!connected_ && !testQuery)
     {
     strcpy(resp_, "");
     return asynSuccess;
     }

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s: controller=\"%s\" command=\"%s\"\n", functionName, address_, cmd_);

  //Append carriage return to provided cmd
  len = strlen(cmd_);
  if (len < MAX_GALIL_STRING_SIZE - 2)
     {
     cmd_[len] = '\r';
     cmd_[len+1] = '\0';
     }
  else //Command too long
     return asynError;

  //Write command, and retrieve response
  status = sync_writeReadController(cmd_, resp_, MAX_GALIL_STRING_SIZE, &nread, timeout_);

  //Remove trailing \r we added earlier
  cmd_[len] = '\0';

  //Remove any unwanted characters
  string resp = resp_;
  resp.erase(resp.find_last_not_of(" \n\r\t:")+1);
  strcpy(resp_, resp.c_str());

  //Debugging
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
         "%s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      functionName, address_, cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));

  if (debug_file != NULL)
     {
     time_t now;
     //Use line buffering, then flush
     setvbuf(debug_file, NULL, _IOLBF, BUFSIZ);
     time(&now);
     char time_buffer[64];
     strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
     fprintf(debug_file, "%s (%d) %s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      time_buffer, getpid(), functionName, address_, cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));
     }

  return (asynStatus)status;
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus GalilController::sync_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  unsigned i = 0;	//Number of raw bytes received, general counting
  unsigned j = 0;	//Number of unsolicited bytes received
  unsigned k = 0;	//Number of solicited bytes received
  size_t nwrite;	//Bytes written
  asynStatus status = asynSuccess;//Asyn status
  int eomReason;	//End of message reason
  char buf[MAX_GALIL_DATAREC_SIZE] = "";	//Receive buffer
  char mesg[MAX_GALIL_DATAREC_SIZE] = "";	//Unsolicited buffer
  char resp[MAX_GALIL_DATAREC_SIZE] = "";	//Solicited buffer
				//Sometimes caller puts many commands on one line separated by ; so we must
  string out_string = output;	//Determine number of output terminators to search for from requested command
  int target_terminators = (int)count(out_string.begin(), out_string.end(), ';') + 1;
  int found_terminators = 0;	//Terminator characters found so far
  unsigned char value;		//Used to identify unsolicited traffic
  bool done = false;		//Read complete?

  //Null user supplied input buffer
  strcpy(input, "");
  //Set timeout for Sync connection
  pasynUserSyncGalil_->timeout = timeout_;
  //Write the command
  status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, output, strlen(output), &nwrite);
  //If write ok
  if (!status)
     {
     while (!done)
        {
        //Read any response
        status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_STRING_SIZE, nread, &eomReason);
        //If read successful, search for terminator characters
        if (!status && *nread > 0)
           {
           //Search for terminating characters
           for (i = 0; i < *nread; i++)
              {
              //Controller responds with ? or : for each command separated by ;
              if (buf[i] == '?')
                 {
                 //Look for command fail prompts
                 found_terminators++;
                 //Controller could not honour command
                 status = asynError;
                 }
              //Look for command success prompts
              if (buf[i] == ':')
                 found_terminators++;
              //Split received byte stream into solicited, and unsolicited messages
              //Unsolicited messages are received here only in synchronous mode
              value = (unsigned char)buf[i] - 128;
              if (((buf[i] & 0x80) == 0x80) && (my_isascii((int)value)))
                 {
                 //Byte looks like an unsolicited packet
                 //Check for overrun
                 if (j > MAX_GALIL_DATAREC_SIZE - 2)
                    return asynError;//No unsolicited message should be this long return error
                 //Copy potential unsolicited byte into mesg buffer
                 mesg[j++] = (unsigned char)value;
                 //Terminate the buffers
                 mesg[j] = '\0';
                 }
              else
                 {
                 //Byte looks like a solicited packet
                 //Check for overrun
                 if (k > MAX_GALIL_DATAREC_SIZE - 2)
                    return asynError;//No solicited message should be this long return error
                 resp[k++] = buf[i];//Byte is part of solicited message
                 //Terminate the buffer
                 resp[k] = '\0';
                 }
              //If received all expected terminators, read is complete
              if (found_terminators == target_terminators)
                 {
                 //Don't attempt any more reads
                 done = true;
                 //stop searching this read, and return the resp, then send unsolicited mesg
                 break;
                 }
              }
           //Copy solicited response back into input buffer supplied
           if (k != 0)
              {
              sprintf(input, "%s%s", input, resp);
              resp[0] = '\0';
              k = 0;
              }
           }
        else //Stop read if any asyn error
           return asynError;
        }//while (!done)
     //Send unsolicited mesg to queue
     if (j != 0)
        sendUnsolicitedMessage(mesg);
     }//write ok
  return status;
}

/** Writes a string to the controller and reads the response.
  * Calls async_writeReadController() with default locations of the input and output strings
  * and default timeout. */ 
asynStatus GalilController::async_writeReadController(void)
{
  size_t nread;
  int status;

  status = async_writeReadController(asynccmd_, asyncresp_, MAX_GALIL_STRING_SIZE, &nread, timeout_);

  //Remove unwanted characters
  string asyncresp = asyncresp_;
  asyncresp.erase(asyncresp.find_last_not_of(" \n\r\t:")+1);
  strcpy(asyncresp_, asyncresp.c_str());

  return (asynStatus)status;
}

/** Writes a string to the Musst controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus GalilController::async_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";

  status = pasynOctetSyncIO->writeRead(pasynUserAsyncGalil_, output,
                                       strlen(output), input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);

  return status;
}

/** Downloads program to controller
*/
asynStatus GalilController::programDownload(string prog)
{
  size_t nwrite;	//Asyn number of bytes written
  size_t nread;		//Asyn number of bytes read
  asynStatus status = asynError;	//Asyn status
  char buf[MAX_GALIL_STRING_SIZE];	//Read back response controller gives at end
  int eomReason;	//end of message reason when reading

  if (connected_)
     {
     //Request download
     status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, "DL\r", 3, &nwrite);
     //Insert download terminate character at program end
     prog.push_back('\\');
     //Download the program
     if (!status)
        {
        status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, prog.c_str(), prog.length(), &nwrite);
        if (!status)  //Read "::" response that controller gives
           status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, (char *)buf, 2, &nread, &eomReason);
        if (buf[0] == '?' || buf[1] == '?')
           status = asynError;  //Controller didn't like the program
        }
     }

  return status;
}

/** Reads array data from controller, and pushes it to database waveform records
*/
asynStatus GalilController::arrayUpload(void)
{
  size_t nwrite;		//Number of characters written
  size_t nread;			//Number of characters read
  asynStatus status = asynError;//Status
  int eomReason;		//End of message reason
  bool done;			//Done reading this array
  string arrayData;		//Data read from controller
  unsigned i, j;		//Looping
  char cmd[MAX_GALIL_STRING_SIZE];//Array upload command
  char buf[MAX_GALIL_STRING_SIZE];//Array upload raw data
  char mesg[MAX_GALIL_STRING_SIZE];//Controller message
  char arrayName[MAX_FILENAME_LEN];//Array name on controller to upload

  //Clear controller message
  setCtrlError("");

  //Update arrayUpload status in paramList
  setIntegerParam(GalilUserArrayUpload_, 1);
  callParamCallbacks();

  if (connected_)
     {
     //Loop through arrays
     for (j = 0;j < 8; j++)
        {
        getStringParam(j, GalilUserArrayName_, (int)sizeof(arrayName), arrayName);
        //Decide to skip or process
        if (!strlen(arrayName)) continue;
        //Request upload
        sprintf(cmd, "QU %s[]\r", arrayName);
        //Called without lock
        lock();
        status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, cmd, 12, &nwrite);
        done = false;
        //Read the response only if write ok
        if (!status)
           {
           while (!done)
              {
              //Read any response
              status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_STRING_SIZE, &nread, &eomReason);
              if (!status && nread > 0)
                 {
                 //Search for error and terminating characters
                 for (i = 0; i < nread; i++)
                    {
                    if (buf[i] == '?')
                       {
                       //Error
                       done = true;
                       sprintf(mesg, "Array upload failed, array name %s is unknown", arrayName);
                       setCtrlError(mesg);
                       callParamCallbacks();
                       status = asynError;
                       }
                    if (i > 0)//terminating characters
                       done = (buf[i-1] == 26 && buf[i] == ':') ? true : false;
                    if (done)
                       break; //Upload complete
                    }
                 if (!status)
                    arrayData.append(buf, i);//Append the upload to arrayData buffer
                 }
              else
                 {
                 done = true; //Stop if any asyn error
                 status = asynError;
                 }
              }//While
           //Release the mutex whilst converting the data
           unlock();
           if (!status)
              {
              //Convert \r separated string of numbers into array of doubles
              stringstream ss(arrayData);
              vector<double> uarray;
              double num;
              while (ss >> num)
                 uarray.push_back(num);
              //Update user array waveform in database
              doCallbacksFloat64Array(&uarray[0], uarray.size(), GalilUserArray_, j);
              //Clear the array data buffer
              arrayData.clear();
              }
           }
        }//For
     }

  //Update arrayUpload status in paramList
  setIntegerParam(GalilUserArrayUpload_, 0);
  callParamCallbacks();
  //Return status to caller
  return status;
}

/** Uploads program on controller and returns as std::string to caller
*/
asynStatus GalilController::programUpload(string *prog)
{
  size_t nwrite;
  size_t nread;
  asynStatus status = asynError;
  int eomReason;
  bool done = false;
  unsigned i;
  char buf[MAX_GALIL_STRING_SIZE];

  //Clear program buffer
  prog->clear();

  //Set timeout for Sync connection
  pasynUserSyncGalil_->timeout = timeout_;

  if (connected_)
     {
     //Request upload
     status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, "UL\r", 3, &nwrite);

     //Read the response only if write ok
     if (!status)
        {
        while (!done)
           {
           //Read any response
           status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_STRING_SIZE, &nread, &eomReason);
           if (!status && nread > 0)
              {
              //Search for terminating : character
              for (i = 0; i < nread; i++)
                 {
                 if (i > 0)
                    done = (buf[i-1] == 26 && buf[i] == ':') ? true : false;
                 if (done)
                    break; //Upload complete
                 }
              //Append the upload to prog buffer
              prog->append(buf, i);
              }
           else
              return asynError; //Stop read if any asyn error
           }

        //Upload complete, finish up
        if (!status && done && prog->length() > 0)
           {
           //Upload appears successful
           //Trim uploaded program
           prog->erase(prog->length()-3);
           }
        }
     }

   //Return status to caller
   return status;
}

/** Initializes variables on the controller
  * \param[in] burn_variables to controller
*/
int GalilController::GalilInitializeVariables(bool burn_variables)
{
   int status = 0;		//Return status			
   int homed[MAX_GALIL_AXES];	//Backup of homed status
   int cinit[MAX_GALIL_AXES];	//Backup of commutation initialized status
   unsigned i;			//General purpose looping
   GalilAxis *pAxis;		//GalilAxis

   if (rio_) //This is a no-op for RIO
      return asynSuccess;

   //Controller wide variables
   //Set tcperr counter to 0
   sprintf(cmd_, "tcperr=0");
   status = sync_writeReadController();

   //Set cmderr counter to 0
   sprintf(cmd_, "cmderr=0");
   status |= sync_writeReadController();

   //Activate input interrupts for motor interlock function
   if (digitalinput_init_ && digports_)
      sprintf(cmd_, "dpon=%d;dvalues=%d;mlock=1", digports_, digvalues_);
   else
      sprintf(cmd_, "dpon=%d;dvalues=%d;mlock=0", digports_, digvalues_);
   sync_writeReadController();

   //Before burning variables backup the commutation initialized, and homed status flags
   //Then set status flags to 0 ready for burn to eeprom
   //Done so commutation initialized and homed status is always 0 at controller power on
   //Finally set axis variables
   if (numAxes_ > 0 && !status)
      {
      for (i = 0;i < numAxes_;i++)
         {
         //Set axis variables
         //Query homed status for this axis
         sprintf(cmd_, "MG homed%c", axisList_[i]);
         if (sync_writeReadController() != asynSuccess)
            {
            //Controller doesnt know homed variable.  
            //Give it initial value only in this case
            sprintf(cmd_, "homed%c=0",  axisList_[i]);
            status |= sync_writeReadController();
            homed[axisList_[i] - AASCII] = 0;
            burn_variables = 1;
            }
         else
            homed[axisList_[i] - AASCII] = atoi(resp_); //Retrieve the homed status
         //Retrieve the axis this homed flag relates to
         pAxis = getAxis(axisList_[i] - AASCII);	//GalilAxis
         if (pAxis)
            {
            //Set motorRecord MSTA bit 15 motorStatusHomed_
            //Homed is not part of Galil data record, we support it using Galil code and unsolicited messages instead
            //We must use asynMotorAxis version of setIntegerParam to set MSTA bits for this MotorAxis
            pAxis->setIntegerParam(motorStatusHomed_, homed[axisList_[i] - AASCII]);
            }
         //Set homed = 0 before burning variables
         sprintf(cmd_, "homed%c=0", axisList_[i]);
         status |= sync_writeReadController();

         //Query commutation initialized status for this axis
         sprintf(cmd_, "MG cinit%c", axisList_[i]);
         if (sync_writeReadController() != asynSuccess)
            {
            //Controller doesnt know commutation initialized variable.  
            //Give it initial value only in this case
            sprintf(cmd_, "cinit%c=0",  axisList_[i]);
            status |= sync_writeReadController();
            cinit[axisList_[i] - AASCII] = 0;
            burn_variables = 1;
            }
         else
            cinit[axisList_[i] - AASCII] = atoi(resp_); //Retrieve the commutation initialized status
         //Set commutation initialized status = 0 before burning variables
         sprintf(cmd_, "cinit%c=0", axisList_[i]);
         status |= sync_writeReadController();

         //provide sensible default for limdc (limit deceleration) value
         sprintf(cmd_, "limdc%c=%ld", axisList_[i], maxAcceleration_);
         status |= sync_writeReadController();
         //Initialize home related parameters on controller
         //initialise home variable for this axis, set to not homming just yet.  Set to homming only when doing a home
         sprintf(cmd_, "home%c=0", axisList_[i]);
         status |= sync_writeReadController();
         //Initialize home switch active value
         sprintf(cmd_, "hswact%c=0", axisList_[i]);
         status |= sync_writeReadController();
         //Initialize home switch inactive value
         sprintf(cmd_, "hswiact%c=1", axisList_[i]);
         status |= sync_writeReadController();
         //Initialise home jogoff variable
         sprintf(cmd_, "hjog%c=0", axisList_[i]);
         status |= sync_writeReadController();
         //Initialize use encoder if present
         sprintf(cmd_, "ueip%c=0", axisList_[i]);
         status |= sync_writeReadController();
         //Initialize use index
         sprintf(cmd_, "ui%c=0", axisList_[i]);
         status |= sync_writeReadController();
         }
      }

   //Burn variables to EEPROM
   if (burn_variables == 1)
      {				
      sprintf(cmd_, "BV");
      if (sync_writeReadController() != asynSuccess)
         {
         status |= asynError;
         errlogPrintf("Error burning variables to EEPROM model %s, address %s\n",model_, address_);
         }
      else
         errlogPrintf("Burning variables to EEPROM model %s, address %s\n",model_, address_);
      }

   //Restore previous homed, and commutation initialized status
   if (numAxes_ > 0)
      {
      for (i = 0;i < numAxes_;i++)
         {
         //Set homed to previous value
         sprintf(cmd_, "homed%c=%d", axisList_[i], homed[axisList_[i] - AASCII]);
         status |= sync_writeReadController();
         //Set commutation initialized to previous value
         sprintf(cmd_, "cinit%c=%d", axisList_[i], cinit[axisList_[i] - AASCII]);
         status |= sync_writeReadController();
         }
      }

   if (status)
      errlogPrintf("Error setting variables on model %s, address %s\n",model_, address_);

   //Return status
   return status;
}

/*--------------------------------------------------------------*/
/* Start the card requested by user   */
/*--------------------------------------------------------------*/
void GalilController::GalilStartController(char *code_file, int burn_program, int thread_mask)
{
   int status;			//Status
   char mesg[MAX_GALIL_STRING_SIZE];
   GalilAxis *pAxis;		//GalilAxis
   unsigned i;			//General purpose looping
   bool start_ok = true;	//Have the controller threads started ok
   int download_status = 0;	//Code download status default = not required
   bool variables_ok = false;	//Were the galil code variables set successfully
   bool burn_variables;		//Burn variables to controller
   string uc;			//Uploaded code from controller
   string dc;			//Code to download to controller
   int display_code = 0;	//For debugging
				//Set bit 1 to display generated code and or the code file specified
				//Set bit 2 to display uploaded code

   //Backup parameters used by developer for later re-start attempts of this controller
   //This allows full recovery after disconnect of controller
   strncpy(code_file_, code_file, sizeof(code_file_));
   code_file_[sizeof(code_file_) - 1] = '\0';
   burn_program_ = burn_program;
   thread_mask_ = thread_mask;

   //Assemble code for download to controller.  This is generated, or user specified code.
   if (!code_assembled_)
      {
      //Assemble the code generated by GalilAxis, if we havent already
      //Assemble code for motor controllers only, not rio
      if (!rio_)
         {
         //First add termination code to end of code generated for this card
         gen_card_codeend();

         //Assemble the sections of generated code for this card
         strcat(card_code_, thread_code_);
         strcat(card_code_, limit_code_);
         strcat(card_code_, digital_code_);
         //Dump generated codefile, which we may or may not actually use
         write_gen_codefile("_gen");
         }

      //load up code file specified by user, if any
      if (!read_codefile(code_file))
         {
         //Copy the user code into card code buffer
         //Ready for delivery to controller
         strcpy(card_code_, user_code_);
         }
      else
         thread_mask_ = (rio_) ? thread_mask_ : 0;  //DMC forced to use generated code

      //Dump card_code_ to file
      write_gen_codefile("_prd");
      }

   //Print out the generated/user code for the controller
   if ((display_code == 1) || (display_code == 3))
      {
      printf("\nGenerated/User code is\n\n");
      cout << card_code_ << endl;
      }

   //If connected, then proceed
   //to check the program on dmc, and download if needed
   if (connected_)
      {
      //Increase timeout whilst manipulating controller code
      timeout_ = 5;
      //Upload code currently in controller for comparison to generated/user code
      status = programUpload(&uc);
      if (status) //Upload failed
         errlogPrintf("\nError uploading code model %s, address %s\n",model_, address_);

      if ((display_code == 2) || (display_code == 3))
         {
         //Print out the uploaded code from the controller
         printf("\nUploaded code is\n\n");
         cout << uc << endl;
         }

      //Uploaded code
      //Remove the \r characters - \r\n is returned by galil controller
      uc.erase (std::remove(uc.begin(), uc.end(), '\r'), uc.end());
      //Change \n to \r (Galil Communications Library expects \r separated lines)
      std::replace(uc.begin(), uc.end(), '\n', '\r');
      //Some controllers dont finish upload with \r\n, ensure buffer ends in carriage return
      if (uc.back() != 13)
         uc.push_back('\r');
      //Download code
      //Copy card_code_ into download code buffer
      dc = card_code_;
      //Remove the \r characters from download code
      dc.erase (std::remove(dc.begin(), dc.end(), '\r'), dc.end());
      //Change \n to \r for controller
      std::replace(dc.begin(), dc.end(), '\n', '\r');

      //If code we wish to download differs from controller current code then download the new code
      if (dc.compare(uc) != 0 && dc.compare("") != 0)
         {
         errlogPrintf("\nTransferring code to model %s, address %s\n",model_, address_);		
         //Do the download
         status = programDownload(dc);
         if (status)
            {
            //Download fail
            sprintf(mesg, "Error downloading code model %s, address %s\n",model_, address_);
            setCtrlError(mesg);
            download_status = -1;
            }
         else
            download_status = 1; //Download success
	
         if (download_status == 1)
            {
            errlogPrintf("Code transfer successful to model %s, address %s\n",model_, address_);	
            //Burn program code to eeprom if burn_program is 1
            if (burn_program == 1)
               {
               //Burn program to EEPROM
               sprintf(cmd_, "BP");
               if (sync_writeReadController() != asynSuccess)
                  errlogPrintf("Error burning code to EEPROM model %s, address %s\n",model_, address_);
               else
                  errlogPrintf("Burning code to EEPROM model %s, address %s\n",model_, address_);

               //Burn parameters to EEPROM
               sprintf(cmd_, "BN");
               if (sync_writeReadController() != asynSuccess)
                  errlogPrintf("Error burning parameters to EEPROM model %s, address %s\n",model_, address_);
               else
                  errlogPrintf("Burning parameters to EEPROM model %s, address %s\n",model_, address_);
               } //if burn_program
            } //download ok
         } //Code on controller different

      //Initialize galil code variables
      burn_variables = (burn_program && (download_status == 1)) ? true : false;
      variables_ok = GalilInitializeVariables(burn_variables) == asynSuccess ? true : false;

      /*Upload code currently in controller to see whats there now*/              
      status = programUpload(&uc);
      if (!status)   //Remove the \r characters - \r\n is returned by galil controller
         uc.erase (std::remove(uc.begin(), uc.end(), '\r'), uc.end());
      else
         errlogPrintf("\nError uploading code model %s, address %s\n",model_, address_);

      //Start thread 0 if upload reveals code exists on controller
      //Its assumed that thread 0 starts any other required threads on controller
      if ((int)uc.length()>2 && thread_mask_ >= 0)
         {
         //Start thread 0
         sprintf(cmd_, "XQ 0,0");
         if (sync_writeReadController() != asynSuccess)
            errlogPrintf("Thread 0 failed to start on model %s address %s\n\n",model_, address_);

         //Wait a second before checking thread status
         epicsThreadSleep(1);
			
         //Check threads on controller
         if (thread_mask_ > 0) //user specified a thread mask, only check for these threads
            {
            for (i = 0; i < 32; ++i)
               {
               if ( (thread_mask_ & (1 << i)) != 0 )
                  {
                  //Check that code is running
                  sprintf(cmd_, "MG _XQ%d", i);
                  if (sync_writeReadController() == asynSuccess)
                     {
                     if (atoi(resp_) == -1)
                        {
                        start_ok = false;
                        sprintf(mesg, "Thread %d failed to start on model %s, address %s\n", i, model_, address_);
                        setCtrlError(mesg);
                        }
                     }
                  }
               }
            }
         else if ((numAxes_ > 0 || rio_) && thread_mask_ == 0) //Check code is running for all created GalilAxis
            {
            if (rio_) //Check thread 0 on rio
               numAxes_ = 1;
            for (i = 0;i < numAxes_;i++)
               {		
               //Check that code is running
               if (rio_)
                  sprintf(cmd_, "MG _XQ%d", i);
               else
                  sprintf(cmd_, "MG _XQ%d", axisList_[i] - AASCII);
               if (sync_writeReadController() == asynSuccess)
                  {
                  if (atoi(resp_) == -1)
                     {
                     start_ok = false;
                     if (rio_)
                        sprintf(mesg, "Thread %d failed to start on model %s, address %s\n", i, model_, address_);
                     else
                        sprintf(mesg, "Thread %d failed to start on model %s, address %s\n", axisList_[i] - AASCII, model_, address_);
                     setCtrlError(mesg);
                     }
                  }
               }
            }

	 if (start_ok)
            errlogPrintf("Code started successfully on model %s, address %s\n",model_, address_);
         }

      //Limits motor direction consistency unknown
      for (i = 0; i < numAxes_; i++)
         {
         pAxis = getAxis(axisList_[i] - AASCII);
         if (!pAxis) continue;
         pAxis->limitsDirState_ = unknown;
         }

      //Retrieve controller time base
      sprintf(cmd_, "MG _TM");
      if (sync_writeReadController() == asynSuccess)
         timeMultiplier_ = DEFAULT_TIME / atof(resp_);

      //Decrease timeout now finished manipulating controller code
      timeout_ = 1;
      //Wake poller, and re-start async records if needed
      poller_->wakePoller();

      }//connected_
   else
      start_ok = false;

   //Set controller start status
   if (start_ok && variables_ok && (download_status == 1 || download_status == 0))
      start_ok = true;
   else
      start_ok = false;
   setIntegerParam(GalilStart_, (int)start_ok);

   //Code is assembled.  Free RAM, and set flag accordingly
   //Keep card_code_ for re-connection/re-start capability
   //Keep card_code_ so we can call GalilStartController internally for re-start
   if (!code_assembled_)
      {
      //free RAM
      free(thread_code_);
      free(limit_code_);
      free(digital_code_);
      free(user_code_);
      //The GalilController code is fully assembled, and stored in GalilController::card_code_
      code_assembled_ = true;
      }
}

/*--------------------------------------------------------------------------------*/
/* Generate code end, and controller wide code eg. error recovery*/

void GalilController::gen_card_codeend(void)
{
   unsigned i;
   struct Galilmotor_enables *motor_enables = NULL;  //Convenience pointer to GalilController motor_enables[digport]

   //Calculate the digports and digvalues required for motor interlock function
   for (i = 0; i < 8; i++)
      {
      //Retrieve structure for digital port from controller instance
      motor_enables = (Galilmotor_enables *)&motor_enables_[i];
      //If this digital in port has at least 1 motor, include it
      if (strlen(motor_enables->motors) > 0)
         {
         //Include the port
         digports_ = digports_ | (1 << i);
         //Include the disable value 
         digvalues_ = digvalues_ | (motor_enables->disablestates[0] << i);
         }
      }

   //generate code end, only if axis are defined	
   if (numAxes_ != 0)
      {
      // Add galil program termination code
      if (digitalinput_init_ == true)
         {
         //Limit code has been written, and we are done with LIMSWI but its not prog end
         strcat(limit_code_, "RE 1\n");
         //Add controller wide motor interlock code to #ININT
         if (digports_ != 0)
            gen_motor_enables_code();
	
         // Add code to end digital port interrupt routine, and end the prog
         strcat(digital_code_, "RI 1\nEN\n");	
         }
      else  //Limit code has been written, and we are done with LIMSWI and its prog end
         strcat(limit_code_, "RE 1\nEN\n");
					
      //Add command error handler
      sprintf(thread_code_, "%s#CMDERR\nerrstr=_ED;errcde=_TC;cmderr=cmderr+1\nEN\n", thread_code_);

      //Add tcp error handler
      sprintf(thread_code_, "%s#TCPERR\ntcpcde=_TC;tcperr=tcperr+1\nRE\n", thread_code_);
      }
}

/*--------------------------------------------------------------------------------*/
/* Generate code to stop motors if disabled via digital IO transition*/
/* Generates code for #ININT */ 
/*
   See also GalilAxis::gen_digitalcode, it too generates #ININT code
*/

void GalilController::gen_motor_enables_code(void)
{
	int i,j;
	struct Galilmotor_enables *motor_enables = NULL;  //Convenience pointer to GalilController motor_enables[digport]
	bool any;
	char c;

	//Assume no digital interlock specified
	any = false;
	
	//Add motor inhibit code for first 8 digital inputs
	for (i=0;i<8;i++)
		{
		//Retrieve structure for digital port from controller instance
		motor_enables = (Galilmotor_enables *)&motor_enables_[i];
		//Generate if statement for this digital port
		if (strlen(motor_enables->motors) > 0)
			{
			any = true;
			sprintf(digital_code_,"%sIF (@IN[%d]=%d)\n", digital_code_, (i + 1), (int)motor_enables->disablestates[0]);
			//Scan through all motors associated with the port
			for (j=0;j<(int)strlen(motor_enables->motors);j++)
				{
				c = motor_enables->motors[j];
				//Add code to stop the motors when digital input state matches that specified
				sprintf(digital_code_,"%sST%c;DC%c=limdc%c;", digital_code_, c, c, c);
				}
			//Manipulate interrupt flag to turn off the interrupt on this port for one threadA cycle
			sprintf(digital_code_,"%s\ndpoff=dpoff-%d;ENDIF\n", digital_code_, (int)pow(2.0,i));
			}
		}
	/* Re-enable input interrupt for all except the digital port(s) just serviced during interrupt routine*/
	/* ThreadA will re-enable the interrupt for the serviced digital ports(s) after 1 cycle */
	if (any)
		strcat(digital_code_,"II ,,dpoff,dvalues\n");
}

/*-----------------------------------------------------------------------------------*/
/*  Dump galil code generated for this controller to file
*/

void GalilController::write_gen_codefile(const char* suffix)
{
	FILE *fp;
	int i = 0;
	char filename[100];
	
	sprintf(filename,"./%s%s.dmc",address_, suffix);
	
	fp = fopen(filename,"wt");

	if (fp != NULL)
		{
		//Dump generated galil code from the GalilController instance
		while (card_code_[i]!='\0')
			{
			fputc(card_code_[i],fp);
			i++;
			}
		fclose(fp);
		}
	else
		errlogPrintf("Could not open for write file: %s",filename);
}

// Load file(s) specified by user into GalilController instance
// The result is stored in GalilController->user_code_ 
// as well as a single code_file, also handles extended syntax of:
//          "header_file;first_axis_file!second_axis_file!third_axis_file;footer_file"
// this allows the downloaded program to be assembed from on-disk templates that are tailored to the
// specific e.g. homing required. Within an axis_file, $(AXIS) is replaced by the relevant axis letter
asynStatus GalilController::read_codefile(const char *code_file)
{
	char* code_file_copy = strdup(code_file); 	//As epicsStrtok_r() modifies string
	char *tokSave = NULL;				//Remaining tokens
	char axis_value[MAX_GALIL_AXES];	//Substitute axis name

	if (strcmp(code_file, "") == 0)
	{	//No code file(s) specified, use generated code
		return asynError;
	}
	//Empty the user code buffer
	user_code_[0] = '\0';
	if (strchr(code_file, ';') == NULL)
	{
		return read_codefile_part(code_file, NULL); // only one part (whole code file specified)
	}
	//Retrieve header file name
	const char* header_file = epicsStrtok_r(code_file_copy, ";", &tokSave);
	if (header_file == NULL)
	{
		errlogPrintf("\nread_codefile: no header file\n\n");
		return asynError;
	}
	//Read the header file
	if (read_codefile_part(header_file, NULL))
		return asynError;
	//Retrieve body file names
	char* body_files = epicsStrtok_r(NULL, ";", &tokSave);
	if (body_files == NULL)
	{
		errlogPrintf("\nread_codefile: no body files\n\n");
		return asynError;
	}
	//Retrieve footer file name
	const char* footer_file = epicsStrtok_r(NULL, ";",  &tokSave);
	if (footer_file == NULL)
	{
		errlogPrintf("\nread_codefile: no footer file\n\n");
		return asynError;
	}
	//Read the body files
	MAC_HANDLE *mac_handle = NULL;
	macCreateHandle(&mac_handle, NULL);
	tokSave = NULL;
	const char* body_file = epicsStrtok_r(body_files, "!", &tokSave);
	for(int i = 0; body_file != NULL; ++i) // i will loop over axis index, 0=A,1=B etc.
	{
		macPushScope(mac_handle);
		//Define the macros we will substitute in the included codefile
		sprintf(axis_value, "%c", i + 'A');
		macPutValue(mac_handle, "AXIS", axis_value);  // substitute $(AXIS) for axis letter 
		//Read the body file
		if (read_codefile_part(body_file, mac_handle))
			return asynError;
		macPopScope(mac_handle);
		//Retrieve the next body file name
		body_file = epicsStrtok_r(NULL, "!", &tokSave);
	}
	macDeleteHandle(mac_handle);
	//Read the footer file
	if (read_codefile_part(footer_file, NULL))
		return asynError;
	//Free the ram we used
	free(code_file_copy);
	return asynSuccess;
}

/*-----------------------------------------------------------------------------------*/
/*  Load the galil code specified into the controller class
*/

asynStatus GalilController::read_codefile_part(const char *code_file, MAC_HANDLE* mac_handle)
{
	int i = 0;
	char file[MAX_FILENAME_LEN];
	FILE *fp;
	//local temp code buffers
	int max_size = MAX_GALIL_AXES * (THREAD_CODE_LEN+LIMIT_CODE_LEN+INP_CODE_LEN);
	char* user_code = (char*)calloc(max_size,sizeof(char));
	char* user_code_exp = (char*)calloc(max_size,sizeof(char));

	if (strcmp(code_file,"")!=0)
		{
		strcpy(file, code_file);

		fp = fopen(file,"rt");

		if (fp != NULL)
			{
			//Read the specified galil code
			while (!feof(fp))
				{
				user_code[i] = fgetc(fp);
				i++;
				}
			fclose(fp);
			user_code[i] = '\0';
	
			//Filter code
			for (i=0;i<(int)strlen(user_code);i++)
				{
				//Filter out any REM lines
				if (user_code[i]=='R' && user_code[i+1]=='E' && user_code[i+2]=='M')
					{
					while (user_code[i]!='\n' && user_code[i]!=EOF)
						i++;
					}
				}
	
			//Terminate the code buffer, we dont want the EOF character
			user_code[i-1] = '\0';
			//Load galil code into the GalilController instance
			if (mac_handle != NULL) // substitute macro definitios for e.g. $(AXIS)
				{
				macExpandString(mac_handle, user_code, user_code_exp, max_size);
				//Copy code into GalilController temporary area
				strcat(user_code_, user_code_exp);
				}
			else
				{
				//Copy code into GalilController temporary area
				strcat(user_code_, user_code);
				}
			}
		else
			{
			if (rio_)
				errlogPrintf("\nread_codefile_part: Can't open user code file \"%s\"\n\n", code_file);
			else
				errlogPrintf("\nread_codefile_part: Can't open user code file \"%s\", using generated code\n\n", code_file);
			return asynError;
			}
		}
	free(user_code);
	free(user_code_exp);
	return asynSuccess;
}

asynStatus GalilController::drvUserCreate(asynUser *pasynUser, const char* drvInfo, const char** pptypeName, size_t* psize)
{
   //const char *functionName = "drvUserCreate";
   char *drvInfocpy;				//copy of drvInfo
   char *charstr;				//The current token
   char *tokSave = NULL;			//Remaining tokens

   //Check if USER_CMD, USER_VAR, USER_OCTET, or USER_OCTET_VAL
   if (strncmp(drvInfo, "USER_", 5) == 0)
     {
     //take a copy of drvInfo
     drvInfocpy = epicsStrDup((const char *)drvInfo);
     //split drvInfocpy into tokens
     //first token is DRVCMD = CMD, OCTET, OCTET_VAL, or VAR
     charstr = epicsStrtok_r((char *)drvInfocpy, " ", &tokSave);
     if (!abs(strcmp(charstr, GalilUserCmdString)))
        pasynUser->reason = GalilUserCmd_;
     if (!abs(strcmp(charstr, GalilUserOctetString)))
        pasynUser->reason = GalilUserOctet_;
     if (!abs(strcmp(charstr, GalilUserOctetValString)))
        pasynUser->reason = GalilUserOctetVal_;
     if (!abs(strcmp(charstr, GalilUserVarString)))
        pasynUser->reason = GalilUserVar_;
     //Second token is GalilStr
     charstr = epicsStrtok_r(NULL, "\n", &tokSave);
     //Store copy of GalilStr in pasynuser userdata
     if (charstr != NULL)
        pasynUser->userData = epicsStrDup(charstr);
     //Free the ram we used
     free(drvInfocpy);
     return asynSuccess;
     }
  else
     {
     return asynMotorController::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
     }
}

asynStatus GalilController::drvUserDestroy(asynUser *pasynUser)
{
   const char *functionName = "drvUserDestroy";
   if ( pasynUser->reason >= GalilUserCmd_ && pasynUser->reason <= GalilUserVar_ )
      {
      asynPrint(pasynUser, ASYN_TRACE_FLOW,
          "%s:%s: index=%d uservar=%s\n", 
          driverName, functionName, pasynUser->reason, (const char*)pasynUser->userData);
      free(pasynUser->userData);
      pasynUser->userData = NULL;
      return(asynSuccess);
      }
  else
      {
      return asynMotorController::drvUserDestroy(pasynUser);
      }
}

/** Record an error message, and also display to ioc window
  * \param[in] mesg      	 Error message
  */
void GalilController::setCtrlError(const char* mesg)
{
   if (mesg[0] != '\0')
      std::cout << mesg << std::endl;
   setStringParam(0, GalilCtrlError_, mesg);
   callParamCallbacks();
}

void GalilController::InitializeDataRecord(void)
{
  int axes;		//Number of axis
  int status;		//Asyn status
  char *charstr;	//The current token
  char *tokSave = NULL;	//Remaining tokens
  char resp[MAX_GALIL_DATAREC_SIZE]; //Copy of response from controller
  int general_b;	//No of general status bytes in data record
  int coord_b;		//No of coordinate system status bytes in data record
  int io_block;		//No of io block bytes in data record
  int axis_b;		//No of axis status bytes in data record
  string model = model_;	//Local copy of model placed into string for easy searching

  //clear the map if there is anything in it
  map.clear();

  //Query for datarecord information
  strcpy(cmd_, "QZ");
  //Ask controller about data record
  status = sync_writeReadController();
  if (!status)
     {
     //Take a local copy of controller response
     strcpy(resp, resp_);
     //Extract number of axes
     charstr = epicsStrtok_r(resp, ",", &tokSave);
     //Ignore above result as QZ reports axes number incorrectly
     //when user issues BA command and uses external amplifier
     //Determine number axes on controller
     axes = numAxesMax_;
     //Extract number of general status bytes
     charstr = epicsStrtok_r(NULL, ",", &tokSave);
     general_b = atoi(charstr);
     //Extract number of coordinate system status bytes
     charstr = epicsStrtok_r(NULL, ",", &tokSave);
     coord_b = atoi(charstr);
     //Extract number of axis status bytes
     charstr = epicsStrtok_r(NULL, ",", &tokSave);
     axis_b = atoi(charstr);
     //Store the data record size
     datarecsize_ = HEADER_BYTES + (axes * axis_b) + general_b + coord_b;
     //DMC300x0 returns 1 18 16 36, search for "DMC31" in model string to determine 16bit ADC
     if (general_b == 18) return Init30010(model.find("DMC31") != string::npos);
     //DMC40x0/DMC41x3/DMC50000         8 52 26 36
     if (axis_b == 36) return Init4000(axes);
     //DMC14x5/2xxx/                 8 24 16 28 //also Optima
     if (axis_b == 28) return Init2103(axes);
     //if here, should be an RIO
     //RIO has a 0 in the axis block data
     if (axis_b != 0) return;
     io_block = coord_b;
     //RIO-47300 has 4 extra bytes in the I/O block
     //RIO-47300 Standard, with Exteneded I/O, with Quad/Biss/SSi
     bool rio3 = ((io_block == 52) || (io_block == 60) || (io_block == 68));
     //SER tacks 4 longs on the end of the data record (4 encoders)
     //471x2,472x2 OR 47300 with SER
     bool rioser = ((io_block == 64) || (io_block == 68));
     //Extended I/O tacks 8 bytes on the end of the data rrecord, three bytes of each of I/O, one padding for each
     //RIO-47300 with extended I/O. Mutually exclusive with SER
     bool rio3ex = (io_block == 60);
     InitRio(rio3);
     if (rio3ex) InitRio3_24Ex();
     if (rioser) InitRioSer(rio3);
     }
}

double GalilController::sourceValue(const std::vector<char>& record, const std::string& source)
{
	try
	{
		const Source& s = map.at(source); //use at() function so silent insert does not occur if bad source string is used.
		int return_value = 0;
		if (s.type[0] == 'U')  //unsigned
			switch (s.type[1])
		{
			case 'B':  return_value = *(unsigned char*)(&record[s.byte]);  break;
			case 'W':  return_value = *(unsigned short*)(&record[s.byte]);  break;
			case 'L':  return_value = *(unsigned int*)(&record[s.byte]);  break;
		}
		else //s.type[0] == 'S'  //signed
			switch (s.type[1])
		{
			case 'B':  return_value = *(char*)(&record[s.byte]);  break;
			case 'W':  return_value = *(short*)(&record[s.byte]);  break;
			case 'L':  return_value = *(int*)(&record[s.byte]);  break;
		}

		if (s.bit >= 0) //this is a bit field
		{
			bool bTRUE = s.scale > 0; //invert logic if scale is <= 0  
			return return_value & (1 << s.bit) ? bTRUE : !bTRUE; //check the bit
		}
		else
			return (return_value / s.scale) + s.offset;

	}
	catch (const std::out_of_range& e) //bad source
	{
		return 0.0;
	}
}

void GalilController::Init30010(bool dmc31010)
{
	char map_address[MAX_GALIL_STRING_SIZE];
	char description[MAX_GALIL_STRING_SIZE];
	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	map["@IN[1]"] = Source(6, "UB", 0, "Boolean", "Digital input 1");
	map["@IN[2]"] = Source(6, "UB", 1, "Boolean", "Digital input 2");
	map["@IN[3]"] = Source(6, "UB", 2, "Boolean", "Digital input 3");
	map["@IN[4]"] = Source(6, "UB", 3, "Boolean", "Digital input 4");
	map["@IN[5]"] = Source(6, "UB", 4, "Boolean", "Digital input 5");
	map["@IN[6]"] = Source(6, "UB", 5, "Boolean", "Digital input 6");
	map["@IN[7]"] = Source(6, "UB", 6, "Boolean", "Digital input 7");
	map["@IN[8]"] = Source(6, "UB", 7, "Boolean", "Digital input 8");

	map["@OUT[1]"] = Source(8, "UB", 0, "Boolean", "Digital output 1");
	map["@OUT[2]"] = Source(8, "UB", 1, "Boolean", "Digital output 2");
	map["@OUT[3]"] = Source(8, "UB", 2, "Boolean", "Digital output 3");
	map["@OUT[4]"] = Source(8, "UB", 3, "Boolean", "Digital output 4");

	map["_TC"] = Source(10, "UB", -1, "", "Error code");

	//Thread status
	map["NO0"] = Source(11, "UB", 0, "Boolean", "Thread 0 running");
	map["NO1"] = Source(11, "UB", 1, "Boolean", "Thread 1 running");
	map["NO2"] = Source(11, "UB", 2, "Boolean", "Thread 2 running");
	map["NO3"] = Source(11, "UB", 3, "Boolean", "Thread 3 running");
	map["NO4"] = Source(11, "UB", 4, "Boolean", "Thread 4 running"); //Firmware prior to 1.2a has only 4 threads
	map["NO5"] = Source(11, "UB", 5, "Boolean", "Thread 5 running");

	//Analog IO
	//version 1.1b provides 16 bit AQ-compatible data in data record
	if (dmc31010)
		aq_analog(12, 2);
	else
		map["@AN[2]"] = Source(12, "UW", -1, "V", "Analog input 2", 13107.2); //0-5 16 bit upsampling

	map["@AO[1]"] = Source(14, "SW", -1, "V", "Analog output 1", 3276.8); //+/- 10v
	map["@AO[2]"] = Source(16, "SW", -1, "V", "Analog output 2", 3276.8);

	//Amp status
	map["TA00"] = Source(18, "UB", 0, "Boolean", "Axis A over current");
	map["TA01"] = Source(18, "UB", 1, "Boolean", "Axis A over voltage");
	map["TA02"] = Source(18, "UB", 2, "Boolean", "Axis A over temperature");
	map["TA03"] = Source(18, "UB", 3, "Boolean", "Axis A under voltage");
	map["TA1A"] = Source(19, "UB", 0, "Boolean", "Axis A hall error");
	map["TA2A"] = Source(20, "UB", 0, "Boolean", "Axis A at _TKA peak current");
	map["TA3AD"] = Source(21, "UB", 0, "Boolean", "Axis A ELO active");

	//contour mode
	map["CD"] = Source(22, "UL", -1, "segments", "Contour segment count");
	map["_CM"] = Source(26, "UW", -1, "elements", "Contour buffer space");

	//S plane
	map["_CSS"] = Source(28, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(30, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(30, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(30, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(31, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(32, "SL", -1, "counts", "Axis S length");
	map["_LMS"] = Source(36, "UW", -1, "elements", "Axis S buffer speace");

	//per-axis data
	int base = 38; //starting offset
	int i = 0; //only one axis on 30010, no need to iterate axes

	map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
	map[ax("HM", i, "3")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " found index"));
	map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
	map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
	map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
	map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
	map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
	map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
	++base; //39
	map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
	map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
	map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
	map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
	map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
	map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
	map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
	map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
	++base; //40
	map[ax("MT", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in stepper mode"));
	map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
	map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
	map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
	//4 and 5 reserved
	map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
	map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
	++base; //41
	map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
	++base; //42
	map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
	base += 4; //46
	map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
	base += 4; //50
	map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
	base += 4; //54
	map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
	base += 4; //58
	map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
	base += 4; //62
	map[ax("_TT", i, "")] = Source(base, "SL", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
	base += 4; //66

	//version 1.1b provides 16 bit AQ-compatible data in data record
	if (dmc31010)
		aq_analog(base, i + 1);
	else
		{
		sprintf(map_address, "@AN[%d]", i + 1);
		sprintf(description, "Analog input %d", i + 1);
		map[map_address] = Source(base, "UW", -1, "V", description, 13107.2);
		}

	base += 2; //68

	map[ax("_QH", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " hall sensors"));
	base++; //69 reserved
	base++; //70
	map[ax("_ZA", i, "")] = Source(base, "SL", -1, "", ax("Axis ", i, " user variable"));
	base += 4; //74
}

void GalilController::Init4000(int axes)
{
	int status;
        int co = -1;

	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	//Digital Inputs
	map["_TI0"] = Source(6, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(6, 1);

	map["_TI1"] = Source(7, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(7, 9);

	//Digital outputs
	map["_OP0"] = Source(16, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(16, 1);

	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(17, 9);

        strcpy(cmd_, "MG_CO");
        status = sync_writeReadController();
        if (!status)
           {
           co = atoi(resp_);
           if (strcmp(resp_, "?") != 0) //41x3 will ? here
                {
		map["_TI2"] = Source(8, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
		map["_TI3"] = Source(9, "UB", -1, "", "Digital inputs 25 to 32");
		map["_TI4"] = Source(10, "UB", -1, "", "Digital inputs 33 to 40");
		map["_TI5"] = Source(11, "UB", -1, "", "Digital inputs 41 to 48");

		map["_OP1"] = Source(18, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
		map["_OP2"] = Source(20, "UW", -1, "", "Digital outputs 33 to 48");

		if (co & 0x00000001) //bank 2 is output
			output_bits(18, 17);
		else //bank 2 in input
			input_bits(8, 17);

		if (co & 0x00000002) //bank 3 is output
			output_bits(19, 25);
		else //bank 3 is input
			input_bits(9, 25);

		if (co & 0x00000004) //bank 4 is output
			output_bits(20, 33);
		else //bank 4 is input
			input_bits(10, 33);

		if (co & 0x00000008) //bank 5 is output
			output_bits(21, 41);
		else
			input_bits(11, 41);
	        }
            }

	//Ethernet Handle Status
	map["_IHA2"] = Source(42, "UB", -1, "", "Handle A Ethernet status");
	map["_IHB2"] = Source(43, "UB", -1, "", "Handle B Ethernet status");
	map["_IHC2"] = Source(44, "UB", -1, "", "Handle C Ethernet status");
	map["_IHD2"] = Source(45, "UB", -1, "", "Handle D Ethernet status");
	map["_IHE2"] = Source(46, "UB", -1, "", "Handle E Ethernet status");
	map["_IHF2"] = Source(47, "UB", -1, "", "Handle F Ethernet status");
	map["_IHG2"] = Source(48, "UB", -1, "", "Handle G Ethernet status");
	map["_IHH2"] = Source(49, "UB", -1, "", "Handle H Ethernet status");

	map["_TC"] = Source(50, "UB", -1, "", "Error code");

	//Thread status
	map["NO0"] = Source(51, "UB", 0, "Boolean", "Thread 0 running");
	map["NO1"] = Source(51, "UB", 1, "Boolean", "Thread 1 running");
	map["NO2"] = Source(51, "UB", 2, "Boolean", "Thread 2 running");
	map["NO3"] = Source(51, "UB", 3, "Boolean", "Thread 3 running");
	map["NO4"] = Source(51, "UB", 4, "Boolean", "Thread 4 running");
	map["NO5"] = Source(51, "UB", 5, "Boolean", "Thread 5 running");
	map["NO6"] = Source(51, "UB", 6, "Boolean", "Thread 6 running");
	map["NO7"] = Source(51, "UB", 7, "Boolean", "Thread 7 running");

	//Amplifier error status
	map["TA00"] = Source(52, "UB", 0, "Boolean", "Axis A-D over current");
	map["TA01"] = Source(52, "UB", 1, "Boolean", "Axis A-D over voltage");
	map["TA02"] = Source(52, "UB", 2, "Boolean", "Axis A-D over temperature");
	map["TA03"] = Source(52, "UB", 3, "Boolean", "Axis A-D under voltage");
	map["TA04"] = Source(52, "UB", 4, "Boolean", "Axis E-H over current");
	map["TA05"] = Source(52, "UB", 5, "Boolean", "Axis E-H over voltage");
	map["TA06"] = Source(52, "UB", 6, "Boolean", "Axis E-H over temperature");
	map["TA07"] = Source(52, "UB", 7, "Boolean", "Axis E-H under voltage");

	map["TA1A"] = Source(53, "UB", 0, "Boolean", "Axis A hall error");
	map["TA1B"] = Source(53, "UB", 1, "Boolean", "Axis B hall error");
	map["TA1C"] = Source(53, "UB", 2, "Boolean", "Axis C hall error");
	map["TA1D"] = Source(53, "UB", 3, "Boolean", "Axis D hall error");
	map["TA1E"] = Source(53, "UB", 4, "Boolean", "Axis E hall error");
	map["TA1F"] = Source(53, "UB", 5, "Boolean", "Axis F hall error");
	map["TA1G"] = Source(53, "UB", 6, "Boolean", "Axis G hall error");
	map["TA1H"] = Source(53, "UB", 7, "Boolean", "Axis H hall error");

	map["TA2A"] = Source(54, "UB", 0, "Boolean", "Axis A at _TKA peak current");
	map["TA2B"] = Source(54, "UB", 1, "Boolean", "Axis B at _TKB peak current");
	map["TA2C"] = Source(54, "UB", 2, "Boolean", "Axis C at _TVC peak current");
	map["TA2D"] = Source(54, "UB", 3, "Boolean", "Axis D at _TKD peak current");
	map["TA2E"] = Source(54, "UB", 4, "Boolean", "Axis E at _TKE peak current");
	map["TA2F"] = Source(54, "UB", 5, "Boolean", "Axis F at _TKF peak current");
	map["TA2G"] = Source(54, "UB", 6, "Boolean", "Axis G at _TKG peak current");
	map["TA2H"] = Source(54, "UB", 7, "Boolean", "Axis H at _TKH peak current");

	map["TA3AD"] = Source(55, "UB", 0, "Boolean", "Axis A-D ELO active");
	map["TA3EH"] = Source(55, "UB", 1, "Boolean", "Axis E-H ELO active");

	//contour mode
	map["CD"] = Source(56, "UL", -1, "segments", "Contour segment count");
	map["_CM"] = Source(60, "UW", -1, "elements", "Contour buffer space");

	//S plane
	map["_CSS"] = Source(62, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(64, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(64, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(64, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(65, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(66, "SL", -1, "counts", "Axis S length");
	map["_LMS"] = Source(70, "UW", -1, "elements", "Axis S buffer speace");

	//T plane
	map["_CST"] = Source(72, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(74, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(74, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(74, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(75, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(76, "SL", -1, "counts", "Axis T length");
	map["_LMT"] = Source(80, "UW", -1, "elements", "Axis T buffer speace");

	//per-axis data
	int base = 82; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("HM", i, "3")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " found index"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //83
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //84
		map[ax("MT", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in stepper mode"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //85
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //86
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //90
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //94
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //98
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //102
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //106
		map[ax("_TT", i, "")] = Source(base, "SL", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 4; //110

		//Analog voltage decoding depends upon AQ setting.
		aq_analog(base, i + 1);
		base += 2; //112

		map[ax("_QH", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " hall sensors"));
		base += 1; //113 reserved
		base += 1; //114
		map[ax("_ZA", i, "")] = Source(base, "SL", -1, "", ax("Axis ", i, " user variable"));
		base += 4; //118

	}// for, axis data
}

void GalilController::Init2103(int axes)
{
	int status;		//Asyn status
	bool db28040 = false;	//DB-28040
	int co = -1;		//Extended IO

	//probe @AN for existance of DB-28040
	strcpy(cmd_, "MG @AN[1]");
	status = sync_writeReadController();
	if (!status)
		if (strcmp(resp_, "?") != 0)
			db28040 = true;

	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");

	//Digital Inputs
	map["_TI0"] = Source(6, "UB", -1, "", "Digital inputs 1 to 8");
	input_bits(6, 1);

	map["_TI1"] = Source(7, "UB", -1, "", "Digital inputs 9 to 16"); //TI always included
	if (axes > 4) //9-16 depend on axes 5-8
		input_bits(7, 9);

	//Digital outputs
	map["_OP0"] = Source(16, "UW", -1, "", "Digital outputs 1 to 16");
	output_bits(16, 1);

	if (axes > 4) //9-16 depend on axes 5-8
		output_bits(17, 9);

	//Extended I/O
	strcpy(cmd_, "MG_CO");
	status = sync_writeReadController();
	if (!status)
		if (db28040 && (strcmp(resp_, "?") != 0))
			{
			map["_TI2"] = Source(8, "UB", -1, "", "Digital inputs 17 to 24"); //TI always included in gcl
			map["_TI3"] = Source(9, "UB", -1, "", "Digital inputs 25 to 32");
			map["_TI4"] = Source(10, "UB", -1, "", "Digital inputs 33 to 40");
			map["_TI5"] = Source(11, "UB", -1, "", "Digital inputs 41 to 48");
			map["_TI6"] = Source(12, "UB", -1, "", "Digital inputs 49 to 56");

			map["_OP1"] = Source(18, "UW", -1, "", "Digital outputs 17 to 32"); //OP always included in gcl
			map["_OP2"] = Source(20, "UW", -1, "", "Digital outputs 33 to 48");
			map["_OP3"] = Source(22, "UW", -1, "", "Digital outputs 49 to 64");

			if (co & 0x00000001) //bank 2 is output
				output_bits(18, 17);
			else //bank 2 in input
				input_bits(8, 17);

			if (co & 0x00000002) //bank 3 is output
				output_bits(19, 25);
			else //bank 3 is input
				input_bits(9, 25);

			if (co & 0x00000004) //bank 4 is output
				output_bits(20, 33);
			else //bank 4 is input
				input_bits(10, 33);

			if (co & 0x00000008) //bank 5 is output
				output_bits(21, 41);
			else //bank 5 is input
				input_bits(11, 41);

			if (co & 0x00000010) //bank 6 is output
				output_bits(22, 49);
			else //bank 6 is input
				input_bits(12, 49);
			}

	map["_TC"] = Source(26, "UB", -1, "", "Error code");

	//general status
	map["_EO"] = Source(27, "UB", 0, "Boolean", "Echo on");
	map["TR"] = Source(27, "UB", 1, "Boolean", "Trace on");
	map["IN"] = Source(27, "UB", 2, "Boolean", "IN waiting for user input");
	map["XQ"] = Source(27, "UB", 7, "Boolean", "Program running");

	//S plane
	map["_CSS"] = Source(28, "UW", -1, "segments", "Axis S segment count");
	map["VDS"] = Source(30, "UB", 3, "Boolean", "Axis S final deceleration");
	map["STS"] = Source(30, "UB", 4, "Boolean", "Axis S stopping");
	map["VSS"] = Source(30, "UB", 5, "Boolean", "Axis S slewing");
	map["_BGS"] = Source(31, "UB", 7, "Boolean", "Axis S moving");
	map["_AVS"] = Source(32, "SL", -1, "counts", "Axis S length");

	//T plane
	map["_CST"] = Source(36, "UW", -1, "segments", "Axis T segment count");
	map["VDT"] = Source(38, "UB", 3, "Boolean", "Axis T final deceleration");
	map["STT"] = Source(38, "UB", 4, "Boolean", "Axis T stopping");
	map["VST"] = Source(38, "UB", 5, "Boolean", "Axis T slewing");
	map["_BGT"] = Source(39, "UB", 7, "Boolean", "Axis T moving");
	map["_AVT"] = Source(40, "SL", -1, "counts", "Axis T length");

	//per-axis data
	int base = 44; //start of A axis data
	for (int i = 0; i < axes; i++)
	{
		map[ax("_MO", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " motor off"));
		map[ax("_OE", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " off-on-error set"));
		map[ax("_AL", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " latch armed"));
		map[ax("DC", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " began deceleration"));
		map[ax("ST", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " began stop"));
		map[ax("SP", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " began slew"));
		map[ax("CM", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in contour mode"));
		map[ax("JG", i, "-")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " negative move"));
		++base; //45
		map[ax("VM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " in VM or LI mode"));
		map[ax("HM", i, "2")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " finding index"));
		map[ax("HM", i, "1")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " coming off home switch"));
		map[ax("HM", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " home command issued"));
		map[ax("FE", i, "")] = Source(base, "UW", 4, "Boolean", ax("Axis ", i, " find edge issued"));
		map[ax("PA", i, "")] = Source(base, "UW", 5, "Boolean", ax("Axis ", i, " in PA mode"));
		map[ax("PR", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " in PA or PR mode"));
		map[ax("_BG", i, "")] = Source(base, "UW", 7, "Boolean", ax("Axis ", i, " move in progress"));
		++base; //46
		map[ax("SM", i, "")] = Source(base, "UW", 0, "Boolean", ax("Axis ", i, " stepper jumper installed"));
		map[ax("_HM", i, "")] = Source(base, "UW", 1, "Boolean", ax("Axis ", i, " home switch"));
		map[ax("_LR", i, "")] = Source(base, "UW", 2, "Boolean", ax("Axis ", i, " reverse limit switch"));
		map[ax("_LF", i, "")] = Source(base, "UW", 3, "Boolean", ax("Axis ", i, " forward limit switch"));
		//4 and 5 reserved
		map[ax("AL", i, "")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch digital input"));
		map[ax("_AL", i, "=0")] = Source(base, "UW", 6, "Boolean", ax("Axis ", i, " latch occurred"));
		++base; //47
		map[ax("_SC", i, "")] = Source(base, "UB", -1, "", ax("Axis ", i, " stop code"));
		++base; //48
		map[ax("_RP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " reference position"));
		base += 4; //52
		map[ax("_TP", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " encoder position"));
		base += 4; //56
		map[ax("_TE", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " position error"));
		base += 4; //60
		map[ax("_TD", i, "")] = Source(base, "SL", -1, "counts", ax("Axis ", i, " dual encoder position"));
		base += 4; //64
		map[ax("_TV", i, "")] = Source(base, "SL", -1, "counts/s", ax("Axis ", i, " filtered velocity"), 64);
		base += 4; //68
		map[ax("_TT", i, "")] = Source(base, "SW", -1, "V", ax("Axis ", i, " torque (DAC)"), 3255);
		base += 2; //70

		if (db28040) //card has onboard Analog inputs
		{
			aq_analog(base, i + 1); //map in the analog
		}
		base += 2; //72
	} //for
}

void GalilController::InitRio(bool rio3)
{

	int status;
	bool aqdq = false;
	bool dqaq = false;

	//0-3 Header is ignored in GCL

	map["TIME"] = Source(4, "UW", -1, "samples", "Sample counter");
	map["_TC"] = Source(6, "UB", -1, "", "Error code");

	//general status
	map["_EO"] = Source(7, "UB", 0, "Boolean", "Echo on");
	map["TR"] = Source(7, "UB", 1, "Boolean", "Trace on");
	map["IN"] = Source(7, "UB", 2, "Boolean", "IN waiting for user input");
	map["XQ"] = Source(7, "UB", 7, "Boolean", "Program running");

	//look for progammable analog I/O
	strcpy(cmd_, "ID");
	status = sync_writeReadController();
	if (!status)
		{
		string resp = resp_;
		//look for progammable analog outputs
		dqaq = (resp.find("(DQ)") != string::npos);
		//look for progammable analog inputs
		aqdq = (resp.find("(AQ)") != string::npos);
		}

	if (dqaq)
	{
		//Programmable outputs found
		dq_analog(8, 0);
		dq_analog(10, 1);
		dq_analog(12, 2);
		dq_analog(14, 3);
		dq_analog(16, 4);
		dq_analog(18, 5);
		dq_analog(20, 6);
		dq_analog(22, 7);
	}
	else //fixed 0-5V
	{
		map["@AO[0]"] = Source(8, "UW", -1, "V", "Analog output 0", 13107.2, 0);
		map["@AO[1]"] = Source(10, "UW", -1, "V", "Analog output 1", 13107.2, 0);
		map["@AO[2]"] = Source(12, "UW", -1, "V", "Analog output 2", 13107.2, 0);
		map["@AO[3]"] = Source(14, "UW", -1, "V", "Analog output 3", 13107.2, 0);
		map["@AO[4]"] = Source(16, "UW", -1, "V", "Analog output 4", 13107.2, 0);
		map["@AO[5]"] = Source(18, "UW", -1, "V", "Analog output 5", 13107.2, 0);
		map["@AO[6]"] = Source(20, "UW", -1, "V", "Analog output 6", 13107.2, 0);
		map["@AO[7]"] = Source(22, "UW", -1, "V", "Analog output 7", 13107.2, 0);
	}

	if (aqdq)
	{
		//Programmable inputs found
		aq_analog(24, 0);
		aq_analog(26, 1);
		aq_analog(28, 2);
		aq_analog(30, 3);
		aq_analog(32, 4);
		aq_analog(34, 5);
		aq_analog(36, 6);
		aq_analog(38, 7);
	}
	else  //fixed 0-5V
	{
		map["@AN[0]"] = Source(24, "UW", -1, "V", "Analog input 0", 13107.2, 0);
		map["@AN[1]"] = Source(26, "UW", -1, "V", "Analog input 1", 13107.2, 0);
		map["@AN[2]"] = Source(28, "UW", -1, "V", "Analog input 2", 13107.2, 0);
		map["@AN[3]"] = Source(30, "UW", -1, "V", "Analog input 3", 13107.2, 0);
		map["@AN[4]"] = Source(32, "UW", -1, "V", "Analog input 4", 13107.2, 0);
		map["@AN[5]"] = Source(34, "UW", -1, "V", "Analog input 5", 13107.2, 0);
		map["@AN[6]"] = Source(36, "UW", -1, "V", "Analog input 6", 13107.2, 0);
		map["@AN[7]"] = Source(38, "UW", -1, "V", "Analog input 7", 13107.2, 0);
	}

	//Data record diverges here for RIO471/472 and RIO473
	int base = 40;

	//outputs
	map["_OP0"] = Source(base, "UB", -1, "", "Digital ouputs 0-7");
	output_bits(base, 0);
	base++;

	map["_OP1"] = Source(base, "UB", -1, "", "Digital outputs 8-15");
	output_bits(base, 8);
	base++;

	if (rio3)
	{
		map["_OP2"] = Source(base, "UB", -1, "", "Digital outputs 16-23");
		output_bits(base, 16);
		base++;
		base++; //one more byte in IO space
	}

	//inputs
	map["_TI0"] = Source(base, "UB", -1, "", "Digital inputs 0-7");
	input_bits(base, 0);
	base++;

	map["_TI1"] = Source(base, "UB", -1, "", "Digital inputs 8-15");
	input_bits(base, 8);
	base++;

	if (rio3)
	{
		map["_TI2"] = Source(base, "UB", -1, "", "Digital inputs 16-23");
		input_bits(base, 16);
		base++;
		base++; //one more byte in IO space
	}

	//pulse counter
	map["_PC"] = Source(base, "UL", -1, "edges", "Pulse counter");
	base += 4;

	//user vars
	map["_ZC"] = Source(base, "SL", -1, "", "1st user variable");
	base += 4;
	map["_ZD"] = Source(base, "SL", -1, "", "2nd user variable");
	base += 4;
}

void GalilController::InitRio3_24Ex(void)
{
	//Extended I/O tacks 8 bytes on the end of the data record, three bytes of each of I/O, one reserved for each
	map["_OP3"] = Source(60, "UB", -1, "", "Digital outputs 24-31");
	output_bits(60, 24);
	map["_OP4"] = Source(61, "UB", -1, "", "Digital outputs 32-39");
	output_bits(61, 32);
	map["_OP5"] = Source(62, "UB", -1, "", "Digital outputs 40-47");
	output_bits(62, 40);
	//byte 63 is reserved

	map["_TI3"] = Source(64, "UB", -1, "", "Digital inputs 24-31");
	input_bits(64, 24);
	map["_TI4"] = Source(65, "UB", -1, "", "Digital inputs 32-39");
	input_bits(65, 32);
	map["_TI5"] = Source(66, "UB", -1, "", "Digital inputs 40-47");
	input_bits(66, 40);
	//byte 67 is reserved
}

void GalilController::InitRioSer(bool rio3)
{
	//SER tacks 4 longs on the end of the data record (4 encoders)
	int base = rio3 ? 60 : 56; //RIO 47300 base data record is longer than the other RIO products due to 24 i/o standard
	map["_QE0"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE1"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE2"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
	map["_QE3"] = Source(base, "SL", -1, "counts", "encoder position"); base += 4;
}

void GalilController::aq_analog(int byte, int input_num)
{
  //When analog voltage decoding depends upon AQ setting.
  string type; //for interpreting analog as signed/unsigned
  double divisor; //for dividing ADC counts to calc volts
  int val;
  int status;
  char map_address[MAX_GALIL_STRING_SIZE];
  char description[MAX_GALIL_STRING_SIZE];

  //Query analog setting
  sprintf(cmd_, "MG{Z10.0}_AQ%d", input_num);
  status = sync_writeReadController();
  if (!status && (strcmp(resp_, "?") != 0)) //don't add analog if error on AQ
     {
     val = atoi(resp_);
     switch (val)
	{
	case 1: case -1:  divisor = 32768.0 / 5;   type = "SW";  break;   //  -5 to 5  V   -32768 to 32767
	case 3: case -3:  divisor = 65536.0 / 5;   type = "UW";  break;   //   0 to 5  V        0 to 65535
	case 4: case -4:  divisor = 65536.0 / 10;  type = "UW";  break;   //   0 to 10 V        0 to 65535
	case 2: case -2:  default: //AQ 2 is the default value
		          divisor = 32768.0 / 10;  type = "SW";  break;   // -10 to 10 V   -32768 to 32767
	}
     sprintf(map_address, "@AN[%d]", input_num);
     sprintf(description, "Analog input %d", input_num);
     map[map_address] = Source(byte, type, -1, "V", description, divisor);
     }
}

string GalilController::ax(string prefix, int axis, string suffix)
{
	return prefix + (char)('A' + axis) + suffix;
}

void GalilController::input_bits(int byte, int num)
{
	stringstream ss;
	char description[MAX_GALIL_STRING_SIZE];

	for (int i = 0; i < 8; i++)
	{
		ss << "@IN[";
		ss << setw(2) << setfill('0') << right << num;
		ss << "]";
		sprintf(description, "Digital input %d", num);
		map[ss.str()] = Source(byte, "UB", i, "Boolean", description);
		ss.str("");
		num++;
	}
}

void GalilController::output_bits(int byte, int num)
{
	stringstream ss;
	char description[MAX_GALIL_STRING_SIZE];

	for (int i = 0; i < 8; i++)
	{
		ss << "@OUT[";
		ss << setw(2) << setfill('0') << right << num;
		ss << "]";
		sprintf(description, "Digital output %d", num);
		map[ss.str()] = Source(byte, "UB", i, "Boolean", description);
		ss.str("");
		num++;
	}
}

void GalilController::dq_analog(int byte, int input_num)
{
	//When analog voltage decoding depends upon DQ setting.
	string type; //for interpreting analog as signed/unsigned
	double divisor; //for dividing ADC counts to calc volts
	double offset = 0.0;	//Offset for converting to volts
	int val;
	int status;
	char map_address[MAX_GALIL_STRING_SIZE];
	char description[MAX_GALIL_STRING_SIZE];
  
	sprintf(cmd_, "MG{Z10.0}_DQ%d", input_num);
	status = sync_writeReadController();
	//don't add analog if error on AQ
	if (!status && (strcmp(resp_, "?") != 0))
	{
	val = atoi(resp_);
	switch (val)
		{
		case 3: divisor = 32768.0 / 5;   type = "UW";  offset = -5.0; break;   //  -5 to 5  V   -32768 to 32767
		case 1: divisor = 65536.0 / 5;   type = "UW";  break;   //   0 to 5  V        0 to 65535
		case 2: divisor = 65536.0 / 10;  type = "UW";  break;   //   0 to 10 V        0 to 65535
		case 4: default: //DQ 4 is the default value
			divisor = 32768.0 / 10;  type = "UW";  offset = -10.0; break;   // -10 to 10 V   -32768 to 32767
		}
	sprintf(map_address, "@AO[%d]", input_num);
	sprintf(description, "Analog output %d", input_num);
	map[map_address] = Source(byte, type, -1, "V", description, divisor, offset);
	}
}

//IocShell functions

/** Creates a new GalilController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] address      	 The name or address to provide to Galil communication library
  * \param[in] updatePeriod	 The time in ms between datarecords.  Async if controller + bus supports it, otherwise is polled/synchronous.
  */
extern "C" int GalilCreateController(const char *portName, const char *address, int updatePeriod)
{
  new GalilController(portName, address, updatePeriod);
  return(asynSuccess);
}

/** Creates a new GalilAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  * \param[in] axisname      	 The name motor A-H 
  * \param[in] limits_as_home    Home routine will use limit switches for homing, and limits appear to motor record as home switch
  * \param[in] enables_string	 Comma separated list of digital IO ports used to enable/disable the motor
  * \param[in] switch_type	 Switch type attached to digital bits for enable/disable motor
  */
extern "C" asynStatus GalilCreateAxis(const char *portName,        	/*specify which controller by port name */
                         	      char *axisname,                  	/*axis name A-H */
				      int limit_as_home,		/*0=no, 1=yes. Using a limit switch as home*/
				      char *enables_string,		/*digital input(s) to use to enable/inhibit motor*/
				      int switch_type)		  	/*digital input switch type for enable/inhbit function*/
{
  GalilController *pC;
  static const char *functionName = "GalilCreateAxis";

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, portName);
    return asynError;
  }
  
  pC->lock();

  new GalilAxis(pC, axisname, limit_as_home, enables_string, switch_type);

  pC->unlock();
  return asynSuccess;
}

/** Creates multiple GalilCSAxis objects.  Coordinate system axis
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  */
extern "C" asynStatus GalilCreateCSAxes(const char *portName)
{
  GalilController *pC;			//The GalilController
  unsigned i;				//looping
  static const char *functionName = "GalilCreateCSAxes";

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, portName);
    return asynError;
  }

  pC->lock();

  //Create all GalilCSAxis from I to P
  for (i = 0; i < MAX_GALIL_AXES; i++)
     new GalilCSAxis(pC, i + IASCII);

  pC->unlock();

  return asynSuccess;
}

/** Starts a GalilController hardware.  Delivers dmc code, and starts it.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  * \param[in] code_file      	 Code file to deliver to hardware
  * \param[in] burn_program      Burn program to EEPROM
  * \param[in] thread_mask	 Indicates which threads to expect running after code file has been delivered and thread 0 has been started. Bit 0 = thread 0 etc.
  */
extern "C" asynStatus GalilStartController(const char *portName,        	//specify which controller by port name
					   const char *code_file,
					   int burn_program,
					   int thread_mask)
{
  GalilController *pC;
  static const char *functionName = "GalilStartController";

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, portName);
    return asynError;
  }
  pC->lock();
  //Call GalilController::GalilStartController to do the work
  pC->GalilStartController((char *)code_file, burn_program, thread_mask);
  pC->unlock();
  return asynSuccess;
}

extern "C" asynStatus GalilCreateProfile(const char *portName,         /* specify which controller by port name */
                            		 int maxPoints)                /* maximum number of profile points */
{
  GalilController *pC;
  static const char *functionName = "GalilCreateProfile";

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);
  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, portName);
    return asynError;
  }
  pC->lock();
  pC->initializeProfile(maxPoints);
  pC->unlock();
  return asynSuccess;
}

//Register the above IocShell functions
//GalilCreateController iocsh function
static const iocshArg GalilCreateControllerArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilCreateControllerArg1 = {"IP address", iocshArgString};
static const iocshArg GalilCreateControllerArg2 = {"update period (ms)", iocshArgInt};
static const iocshArg * const GalilCreateControllerArgs[] = {&GalilCreateControllerArg0,
                                                             &GalilCreateControllerArg1,
                                                             &GalilCreateControllerArg2};
                                                             
static const iocshFuncDef GalilCreateControllerDef = {"GalilCreateController", 3, GalilCreateControllerArgs};

static void GalilCreateContollerCallFunc(const iocshArgBuf *args)
{
  GalilCreateController(args[0].sval, args[1].sval, args[2].ival);
}

//GalilCreateAxis iocsh function
static const iocshArg GalilCreateAxisArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilCreateAxisArg1 = {"Specified Axis Name", iocshArgString};
static const iocshArg GalilCreateAxisArg2 = {"Limit switch as home", iocshArgInt};
static const iocshArg GalilCreateAxisArg3 = {"Motor enable string", iocshArgString};
static const iocshArg GalilCreateAxisArg4 = {"Motor enable switch type", iocshArgInt};

static const iocshArg * const GalilCreateAxisArgs[] =  {&GalilCreateAxisArg0,
                                                        &GalilCreateAxisArg1,
							&GalilCreateAxisArg2,
							&GalilCreateAxisArg3,
							&GalilCreateAxisArg4};

static const iocshFuncDef GalilCreateAxisDef = {"GalilCreateAxis", 5, GalilCreateAxisArgs};

static void GalilCreateAxisCallFunc(const iocshArgBuf *args)
{
  GalilCreateAxis(args[0].sval, args[1].sval, args[2].ival, args[3].sval, args[4].ival);
}

//GalilCreateVAxis iocsh function
static const iocshArg GalilCreateCSAxesArg0 = {"Controller Port name", iocshArgString};

static const iocshArg * const GalilCreateCSAxesArgs[] =  {&GalilCreateCSAxesArg0};

static const iocshFuncDef GalilCreateCSAxesDef = {"GalilCreateCSAxes", 1, GalilCreateCSAxesArgs};

static void GalilCreateCSAxesCallFunc(const iocshArgBuf *args)
{
  GalilCreateCSAxes(args[0].sval);
}

//GalilCreateProfile iocsh function
static const iocshArg GalilCreateProfileArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilCreateProfileArg1 = {"Max points", iocshArgInt};
static const iocshArg * const GalilCreateProfileArgs[] = {&GalilCreateProfileArg0,
                                                          &GalilCreateProfileArg1};
                                                             
static const iocshFuncDef GalilCreateProfileDef = {"GalilCreateProfile", 2, GalilCreateProfileArgs};

static void GalilCreateProfileCallFunc(const iocshArgBuf *args)
{
  GalilCreateProfile(args[0].sval, args[1].ival);
}

//GalilStartController iocsh function
static const iocshArg GalilStartControllerArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilStartControllerArg1 = {"Code file", iocshArgString};
static const iocshArg GalilStartControllerArg2 = {"Burn program", iocshArgInt};
static const iocshArg GalilStartControllerArg3 = {"Thread mask", iocshArgInt};
static const iocshArg * const GalilStartControllerArgs[] = {&GalilStartControllerArg0,
                                                            &GalilStartControllerArg1,
                                                            &GalilStartControllerArg2,
                                                            &GalilStartControllerArg3};
                                                             
static const iocshFuncDef GalilStartControllerDef = {"GalilStartController", 4, GalilStartControllerArgs};

static void GalilStartControllerCallFunc(const iocshArgBuf *args)
{
  GalilStartController(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

//Construct GalilController iocsh function register
static void GalilSupportRegister(void)
{
  iocshRegister(&GalilCreateControllerDef, GalilCreateContollerCallFunc);
  iocshRegister(&GalilCreateAxisDef, GalilCreateAxisCallFunc);
  iocshRegister(&GalilCreateCSAxesDef, GalilCreateCSAxesCallFunc);
  iocshRegister(&GalilCreateProfileDef, GalilCreateProfileCallFunc);
  iocshRegister(&GalilStartControllerDef, GalilStartControllerCallFunc);
}

//Finally do the registration
extern "C" {
epicsExportRegistrar(GalilSupportRegister);
}

