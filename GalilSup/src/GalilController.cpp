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
// Mark Clift
// email: padmoz@tpg.com.au

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
// 28/02/16 M.Clift
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
// 14/08/17 M.Clift
//                  Fix issue with homing when useSwitch set false
// 02/09/17 M.Clift
//                  Fix issue with NTM on real motors
// 06/09/17 M.Clift
//                  Add stop reason to internal stop mechanism
//                  Tidy up of internal motor stop mechansim
//                  Altered decimal places used in CSAxis velocity checking messages
// 11/09/17 M.Clift
//                  Improve internal stop mechanism so CSAxis stops are fully coordinated
// 18/09/17 M. Pearson
//                  Fix GalilAxis and GalilCSAXis destructors to remove compiler warnings on gcc 4.8.5 and above
// 09/11/17 M. Pearson
//                  Add ability to enable/disable hardware limits
// 16/11/17 M. Pearson
//                  Add support for BiSS encoder configuration and axis/encoder status polling
// 17/11/17 M. Clift
//                  Increase communication timeout for GalilStartController
// 26/11/17 M.Clift
//                  Add QEGUI and MEDM screens for BISS support
//                  Add BISS support PV's to motor extra autosave request
//                  Alter how BISS, and SSI capability detected
//                  Add disable wrong limit protection when an axis limit is disabled
// 27/11/17 M.Clift
//                  Alter how BISS, and SSI capability detected again
// 16/12/17 M.Clift
//                  Add support for EtherCat axis
// 16/01/18 M.Clift & R. Sluiter
//                  Fix motor velocity issue in profile motion when using custom time base
// 18/02/18 M.Clift & M. Pearson
//                  Alter soft limits.  Dial high = Dial low = 0 now disables axis soft limits
// 22/05/18 M.Clift & M. Pearson
//                  Fix DVAL set zero at autosave restore when controller not connected
// 05/06/18 M.Clift
//                  Fix issue detecting gray code SSI encoder connect/disconnect status
// 04/12/18 M.Clift & M. Pearson
//                  Alter home routine now takes limit disable setting into account
// 19/12/18 M.Clift & M. Pearson
//                  Add iocShell function GalilAddCode - Adds custom code to generated code
// 20/12/18 M.Clift
//                  Fix issue initialising galil variables for rio plc's
// 06/01/19 M.Clift
//                  Alter homing check to cater for disabled soft limits
//                  Fix issue with generated home routine on 4000 series controllers and above
// 30/01/19 M.Clift
//                  Fix jog after home going the wrong way with DIR set to 1 (neg)
//                  Alter output compare start and increment/delta now specified in user coordinates
// 02/06/19 M.Clift
//                  Alter homing check so that custom home code can enable/disable limits
//                  Fix output compare increment calculation
//                  Add driver version number
//                  Alter set_biss so that it's windows compatible
//                  Add iocShell function GalilReplaceHomeCode - Replace generated axis home with custom home code
// 04/06/19 M.Clift
//                  Alter digital IO records update now forced upon controller connect
//                  Tidy up GalilConnector
//                  Alter digital IO MEDM and QEGUI screens so digital input alarm state and value is displayed
//                  Alter default alarm states for digital in bits
// 12/05/19 M.Clift
//                  Alter axisStatusThread for faster ioc exit
//                  Fix segmentation fault on exit
//                  Add SP_MON PV to CSAxis for monitoring current setpoint
//                  Add galil_motors-v6-9down substitution and template files for motor record 6-9 and below
//                  Add galil_motors-v6-10up substitution and template files for motor record 6-10 and above
// 30/09/19 M. Pearson
//                  Add an encoder tolerance parameter that is used when determining encoder motion and direction
//                  in the GalilAxis::setStatus function. This helps deal with detecting stall conditions
//                  with high resolution encoders (that may always be changing by a few counts even when stationary).
// 23/10/19 M. Clift
//                  Fix use of constants in kinematics such as PI, D2R, etc
//                  Alter CSAxis limit orientation calculation to improve robustness
//                  Add handler to catch SIGTERM, SIGINT and initiate clean shutdown
// 27/10/19 M. Clift
//                  Improve kinematics messaging
//                  Add encoder tolerance PV to QEGUI, MEDM motor extras screen
// 13/01/2020 M. Clift & M. Pearson
//                  Fix add thread synchronization where required
// 02/02/2020 M. Clift
//                  Replaced some c char arrays with c++ strings
//                  Fix multiple issues with GalilReplaceHomeCode
//                  Fix CSAxis occasionally stops responding
//                  Fix CSAxis was incorrectly setting axis move dynamics when axis move < rdbd
//                  Fix CSAXis moves now if any axis >= 1 step move rather than >= rdbd
//                  Alter group move now rejected if any axis in group fails readiness checks
//                  Fix motor post, auto amp off, and auto brake on now after all backlash, and retries
//                  Improved internal stop mechanism.  Backlash, retries now prevented as necessary
//                  Add analog output readbacks for DMC30000 series
//                  Altered coordinate system stop mechanism for ad-hoc deferred moves
// 23/04/2020 M. Clift
//                  Fix issue with limit switch interrupt routine in generated code
// 06/05/2020 M.Clift
//                  Fix/Add use synchronous comms if sync/tcp connects but async/udp doesn't
// 11/07/2020 M.Clift
//                  Removed motor extras manual brake on/off command
//                  Minor tidy up
// 05/11/2020 M.Clift
//                  Add motor direction / limit consistent indicator to motor extras
// 06/01/2021 M.Clift
//                  Fix home process no longer disables wrong limit protection (wlp)
// 04/02/2021 M.Clift
//                  Add shareLib.h where required for EPICS 7 compatibility
// 15/02/2021 M.Clift
//                  Fix segmentation fault in GalilAddCode
// 28/05/2021 M.Clift
//                  Fix homing to home switch when limit at same location
// 16/07/2021 M.Clift
//                  Add PV's to call GalilAxis:home in addition to motor record homr/homf fields
//                  Improved home when use switch true in specified direction when limit already active
//                  Improved motor limits direction consistency check
// 12/12/2021 M.Clift
//                  Altered kinematic equation records to waveform char[256]
//                  Fix remove spaces from kinematic equations
// 18/12/2021 M.Clift
//                  Fix issue parsing kinematic equations
// 03/09/2022 M.Clift
//                  Add $(P):HOMEEDGE_CMD, Add $(P):HOMEEDGE_STATUS to set home switch edge found during homing
//                  Alter home switch type, limit switch type PV's to be more general
//                  Improved generated home routine for home to home switch
//                  Fix motor acceleration not set issue introduced by $(P)$M)HOMR_CMD/$(P)$M)HOMF_CMD homing PV's
//                  Moved limits as home option to motor extras with PV $(P)$M)ULAH_CMD/$(P)$M)ULAH_STATUS
//                  Improved motor limits direction consistency check
//                  Improved wrong limit protection
//                  Add alarms states to $(P)$(M)_LIMITCONSISTENT_STATUS
//                  Renamed $(P)$(M)_WLPACTIVE_STATUS to $(P)$(M)_WLPSTOP_STATUS
// 28/10/2022 M.Rivers
//                  Add support for asynEnum
//                  Add microstep record to motor extras
//                  Add microstep record to MEDM screen
// 28/10/2022 M.Clift
//                  Split databases, start scripts, autosave setup for DMC and RIO
//                  Add microstep record to QEGUI screen
// 02/07/2023 M.Clift
//                  Fix $(P)OC2AXIS_STATUS record process error when only 1 axis created
//                  Fix CSAxis calculation of reverse axis velocity, acceleration
//                  Fix CSAxis beginGroupMotion error during home when motors aren't allowed to home
// 03/03/2024 M.Clift
//                  Fix compilation error caused by use of "and" keyword. Revert to && for greater compiler compatibility
// 29/08/2024 A.Dalleh
//                  Fix switch interlock code generation creating lines greater than 80 characters
// 01/09/2024 M.Clift
//                  Fix setLowLimit, setHighLimit clearing DMC01:ERROR_MON controller message
//                  Updated QtEPICS, MEDM screens
//                  Revert Limit, Home switch type to NC (normally closed), NO (normally open)
//                  Alter GalilStartController to check for program lines greater than 80 characters
// 07/09/2024 M.Rivers
//                  Merge in support for 3040 and 3140 amplifiers
// 16/10/2024 M.Clift
//                  Fix change text entry widgets to string type in galil_csmotor_kinematics.adl
// 17/10/2024 M.Clift
//                  Add support for motor record SET field to GalilCSAxis
// 13/01/2025 M.Rivers
//                  Fixed report function
//                  Update Galil amplifier support to use switch statement instead
//                  Fix bad string formatting in wlp support
//                  Add variable initialization in GalilCSAxis to remove compile warnings
//                  Add GalilDummy.cpp to create the GalilSupport registrar entry for systems without C++11
//                  Changed src/Makefile to build the real driver if HAVE_C++11 is true (the default) and the dummy driver if it is false.
//                  Added CONFIG_SITE.Common.$(EPICS_HOST_ARCH) for a few systems that don't have C++11
// 13/01/2025 M.Clift
//                  Add acceleration capped at controller maximum for independent moves
//                  Rename config/GALILRELEASE to config/RELEASE.local
// 16/01/2025 M.Clift
//                  Fix unknown amplifier messages at ioc start when no controller
// 17/01/2025 M.Clift
//                  Change user defined record prefix now derived from PORT
// 24/01/2025 R.Riley, M.Rivers
//                  Fix DMOV set true whilst controller still outputting step pulses that occurred
//                  when the step smoothing factor (motor_extras) set higher than controller default
// 10/03/2025 M.Clift
//                  Fix GalilAddCode to remove REM lines and replace empty lines in custom code with '
//                  Fix GalilReplaceHomeCode to remove REM lines and replace empty in custom code lines with '
//                  Fix GalilStartController to remove REM lines and replace empty in custom code lines with '
// 27/03/2025 M.Clift
//                  Change/increase timeout for user console commands known to take a long time
//                  Add command console monitor PV $(P):SEND_STR_MON will now show controller error strings
//                  Fix multi-thread access condition causing segfault during controller reconnect in UDP mode
// 06/05/2025 M.Clift, JHopkins
//                  Add missing config/RELEASE.local
// 27/09/2025 M.Clift, M.Rivers
//                  Add internal amplifier status and fault reset
// 27/09/2025 M.Clift
//                  Add several stop code cases to internal stop mechanism
//                  Minor optimizations to GalilController::getStatus
// 05/10/2025 M.Rivers
//                  Fix data record decoding of amplifier fault status
//                  Update internal amplifer MEDM screen
// 12/10/2025 M.Clift
//                  Add block ST command through command console preventing accidental kill of all controller threads 
//                  Fix some characters missing from unsolicited messages
// 30/10/2025 M.Clift
//                  Move config/RELEASE.local to configure/.  Remove config folder
//                  Fault status colors altered (eg. moving, limits)

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#if defined _WIN32 || _WIN64
#include <process.h>
#else
#include <unistd.h>
#endif /* _WIN32 */
#include <iostream>  //cout
#include <fstream>   //ifstream
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
#include <shareLib.h>
#include <drvAsynIPPort.h>
#include <drvAsynSerialPort.h>

#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <asynOctet.h>
#include <asynShellCommands.h>

#include "GalilController.h"
#include <epicsExport.h>

static const char *driverName = "GalilController";
static const char *driverVersion = "4-1-09";

static void GalilProfileThreadC(void *pPvt);
static void GalilArrayUploadThreadC(void *pPvt);

//Block read functions during Iocinit
//Prevent normal behaviour of output records getting/reading initial value at iocInit from driver
//Instead output records will write their db default or autosave value just after iocInit
//This change in behaviour is anticipated by the asyn record device layer and causes no error mesgs
static bool dbInitialized = false;

//Static count of Galil controllers.  Used to derive communications port name L(controller num)
static int controller_num = 0;

//Signal handler setup for SIGTERM, and SIGINT
static bool signalHandlerSetup = false; 

//Convenience functions
#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

//C++ "To String with Precision" static function template
template <typename T>
string tsp(const T a_value, const int n)
{
    ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

//Signal handler for SIGTERM, and SIGINT
static void signalHandler(int sig)
{
   switch (sig) {
      case SIGINT:
         epicsExit (128 + sig);
         break;
      case SIGTERM:
         epicsExit (128 + sig);
         break;
   }
}

//EPICS shutdown handler
static void shutdownCallback(void *pPvt)
{
  GalilController *pC_ = (GalilController *)pPvt;
  //IOC shutdown is in progress
  pC_->shutdownController();
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
   int connected;           //Asyn connected flag
   string mesg;              //Controller mesg

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
             //Update connection status pv
             pC_->setIntegerParam(pC_->GalilCommunicationError_, 1);
             //If asyn connected = 0 device wont respond so go ahead and set GalilController connected_ false
             pC_->connected_ = false;
             //Force update to digital IO records when connection is established
             pC_->digInitialUpdate_ = false;
             //Give disconnect message
             pC_->lock();
             mesg = "Disconnected from " + pC_->model_ + " at " + pC_->address_;
             pC_->setCtrlError(mesg);
             pC_->unlock();
             }
          }
       }
}

/** Creates a new GalilController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] address      	 The name or address to provide to Galil communication library 
  * \param[in] updatePeriod  	 The time between polls when any axis is moving
                                 If (updatePeriod < 0), polled/synchronous at abs(updatePeriod) is done regardless of bus type 
  */
GalilController::GalilController(const char *portName, const char *address, double updatePeriod)
  :  asynMotorController(portName, (int)(MAX_ADDRESS), (int)NUM_GALIL_PARAMS,
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask | asynEnumMask | asynDrvUserMask), 
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask | asynEnumMask),
                         (int)(ASYN_CANBLOCK | ASYN_MULTIDEVICE),
                         (int)1, // autoconnect
                         (int)0, (int)0),  // Default priority and stack size
  numAxes_(0), unsolicitedQueue_(MAX_GALIL_AXES, MAX_GALIL_STRING_SIZE)
{
  struct Galilmotor_enables *motor_enables = NULL;	//Convenience pointer to GalilController motor_enables[digport]
  string mesg;              //Controller mesg
  unsigned i;

  // Create controller-specific parameters
  createParam(GalilDriverString, asynParamOctet, &GalilDriver_);
  createParam(GalilAddressString, asynParamOctet, &GalilAddress_);
  createParam(GalilModelString, asynParamOctet, &GalilModel_);
  createParam(GalilHomeTypeString, asynParamInt32, &GalilHomeType_);
  createParam(GalilHomeEdgeString, asynParamInt32, &GalilHomeEdge_);
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
  createParam(GalilClearAmpFaultsString, asynParamInt32, &GalilClearAmpFaults_);

  createParam(GalilOutputCompare1AxisString, asynParamInt32, &GalilOutputCompareAxis_);
  createParam(GalilOutputCompare1StartString, asynParamFloat64, &GalilOutputCompareStart_);
  createParam(GalilOutputCompare1IncrString, asynParamFloat64, &GalilOutputCompareIncr_);
  createParam(GalilOutputCompareMessageString, asynParamOctet, &GalilOutputCompareMessage_);

  createParam(GalilCSMotorSetPointString, asynParamFloat64, &GalilCSMotorSetPoint_);
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
  createParam(GalilMotorStopString, asynParamInt32, &GalilMotorRecordStop_);

  createParam(GalilSSIConnectedString, asynParamInt32, &GalilSSIConnected_);	
  createParam(GalilEncoderStallTimeString, asynParamFloat64, &GalilEStallTime_);

  createParam(GalilStepSmoothString, asynParamFloat64, &GalilStepSmooth_);
  createParam(GalilMotorTypeString, asynParamInt32, &GalilMotorType_);
  createParam(GalilBrushTypeString, asynParamInt32, &GalilBrushType_);

  createParam(GalilEtherCatCapableString, asynParamInt32, &GalilEtherCatCapable_);
  createParam(GalilEtherCatNetworkString, asynParamInt32, &GalilEtherCatNetwork_);
  createParam(GalilCtrlEtherCatFaultString, asynParamInt32, &GalilCtrlEtherCatFault_);
  createParam(GalilEtherCatAddressString, asynParamInt32, &GalilEtherCatAddress_);
  createParam(GalilEtherCatFaultString, asynParamInt32, &GalilEtherCatFault_);
  createParam(GalilEtherCatFaultResetString, asynParamInt32, &GalilEtherCatFaultReset_);

  createParam(GalilMotorConnectedString, asynParamInt32, &GalilMotorConnected_);

  createParam(GalilAfterLimitString, asynParamFloat64, &GalilAfterLimit_);
  createParam(GalilWrongLimitProtectionString, asynParamInt32, &GalilWrongLimitProtection_);
  createParam(GalilWrongLimitProtectionStopString, asynParamInt32, &GalilWrongLimitProtectionStop_);

  createParam(GalilUserOffsetString, asynParamFloat64, &GalilUserOffset_);
  createParam(GalilEncoderResolutionString, asynParamFloat64, &GalilEncoderResolution_);
  createParam(GalilUseEncoderString, asynParamInt32, &GalilUseEncoder_);
  createParam(GalilStopPauseMoveGoString, asynParamInt32, &GalilStopPauseMoveGo_);

  createParam(GalilPremString, asynParamOctet, &GalilPrem_);
  createParam(GalilPostString, asynParamOctet, &GalilPost_);

  createParam(GalilUseLimitsAsHomeString, asynParamInt32, &GalilUseLimitsAsHome_);
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
  createParam(GalilMicrostepString, asynParamInt32, &GalilMicrostep_);

  createParam(GalilAmpModelString, asynParamInt32, &GalilAmpModel_);
  createParam(GalilAmpGainString, asynParamInt32, &GalilAmpGain_);
  createParam(GalilAmpCurrentLoopGainString, asynParamInt32, &GalilAmpCurrentLoopGain_);
  createParam(GalilAmpLowCurrentString, asynParamInt32, &GalilAmpLowCurrent_);
  createParam(GalilHomingString, asynParamInt32, &GalilHoming_);
  createParam(GalilHomrString, asynParamInt32, &GalilHomr_);
  createParam(GalilHomfString, asynParamInt32, &GalilHomf_);
  createParam(GalilUserDataString, asynParamFloat64, &GalilUserData_);
  createParam(GalilUserDataDeadbString, asynParamFloat64, &GalilUserDataDeadb_);

  createParam(GalilLimitDisableString, asynParamInt32, &GalilLimitDisable_);
  createParam(GalilLimitConsistentString, asynParamInt32, &GalilLimitConsistent_);
  createParam(GalilEncoderToleranceString, asynParamInt32, &GalilEncoderTolerance_);

  createParam(GalilMainEncoderString, asynParamInt32, &GalilMainEncoder_);
  createParam(GalilAuxEncoderString, asynParamInt32, &GalilAuxEncoder_);
  createParam(GalilMotorAcclString, asynParamFloat64, &GalilMotorAccl_);
  createParam(GalilMotorRdbdString, asynParamFloat64, &GalilMotorRdbd_);
  createParam(GalilMotorHvelString, asynParamFloat64, &GalilMotorHvel_);
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

  createParam(GalilBISSCapableString, asynParamInt32, &GalilBISSCapable_);
  createParam(GalilBISSInputString, asynParamInt32, &GalilBISSInput_);
  createParam(GalilBISSData1String, asynParamInt32, &GalilBISSData1_);
  createParam(GalilBISSData2String, asynParamInt32, &GalilBISSData2_);
  createParam(GalilBISSZPString, asynParamInt32, &GalilBISSZP_);
  createParam(GalilBISSCDString, asynParamInt32, &GalilBISSCD_);
  createParam(GalilBISSLevelString, asynParamInt32, &GalilBISSLevel_);
  createParam(GalilBISSStatTimeoutString, asynParamInt32, &GalilBISSStatTimeout_);
  createParam(GalilBISSStatCRCString, asynParamInt32, &GalilBISSStatCRC_);
  createParam(GalilBISSStatErrorString, asynParamInt32, &GalilBISSStatError_);
  createParam(GalilBISSStatWarnString, asynParamInt32, &GalilBISSStatWarn_);
  createParam(GalilBISSStatPollString, asynParamInt32, &GalilBISSStatPoll_);

  createParam(GalilErrorLimitString, asynParamFloat64, &GalilErrorLimit_);
  createParam(GalilErrorString, asynParamFloat64, &GalilError_);
  createParam(GalilOffOnErrorString, asynParamInt32, &GalilOffOnError_);
  createParam(GalilAxisString, asynParamInt32, &GalilAxis_);
  createParam(GalilMotorVelocityEGUString, asynParamFloat64, &GalilMotorVelocityEGU_);
  createParam(GalilMotorVelocityRAWString, asynParamFloat64, &GalilMotorVelocityRAW_);

  createParam(GalilMotorHallErrorStatusString, asynParamInt32, &GalilMotorHallErrorStatus_);
  createParam(GalilMotorAtTorqueLimitStatusString, asynParamInt32, &GalilMotorAtTorqueLimitStatus_);
  createParam(GalilAmpOverCurrentStatusString, asynParamInt32, &GalilAmpOverCurrentStatus_);
  createParam(GalilAmpUnderVoltageStatusString, asynParamInt32, &GalilAmpUnderVoltageStatus_);
  createParam(GalilAmpOverVoltageStatusString, asynParamInt32, &GalilAmpOverVoltageStatus_);
  createParam(GalilAmpOverTemperatureStatusString, asynParamInt32, &GalilAmpOverTemperatureStatus_);
  createParam(GalilAmpELOStatusString, asynParamInt32, &GalilAmpELOStatus_);

  createParam(GalilUserCmdString, asynParamFloat64, &GalilUserCmd_);
  createParam(GalilUserOctetString, asynParamOctet, &GalilUserOctet_);
  createParam(GalilUserOctetValString, asynParamFloat64, &GalilUserOctetVal_);
  createParam(GalilUserVarString, asynParamFloat64, &GalilUserVar_);

  createParam(GalilEthAddrString, asynParamOctet, &GalilEthAddr_);
  createParam(GalilSerialNumString, asynParamOctet, &GalilSerialNum_);

  createParam(GalilStatusPollDelayString, asynParamFloat64, &GalilStatusPollDelay_);

//Add new parameters here

  createParam(GalilCommunicationErrorString, asynParamInt32, &GalilCommunicationError_);

  //Not connected to controller yet
  connected_ = false;
  //Store address
  address_ = address;
  //Default model
  model_ = "Unknown";
  //Default rio
  rio_ = false;
  //Default numAxesMax_
  numAxesMax_ = 0;
  //Default ampModel_
  ampModel_[0] = 0;
  ampModel_[1] = 0;
  //IOC is not shutting down yet
  shuttingDown_ = false;
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
     mesg = "Capping UpdatePeriod to " + tsp(MAX_UPDATE_PERIOD) + "ms maximum";
     setCtrlError(mesg);
     updatePeriod_ = MAX_UPDATE_PERIOD;
     }
  //Digital in code generator has not been initialized
  digitalinput_init_ = false;
  //Motor enables defaults
  digports_ = 0;
  digvalues_ = 0;
  //Time multiplier default
  timeMultiplier_ = 1.00;
  //Initial update to EPICS digital input records hasn't been done yet
  digInitialUpdate_ = false;
  //Deferred moves off at start-up
  movesDeferred_ = false;
  //Store the controller number for later use
  controller_number_ = controller_num;

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

  //Add code headers to generated galil code
  gen_code_headers();

  //Static count of controllers.  Used to derive communications port names
  controller_num++;

  //Setup signal handler to catch SIGTERM, and SIGINT
  if (!signalHandlerSetup) {
     signal(SIGTERM, signalHandler);
     signal(SIGINT, signalHandler);
     signalHandlerSetup = true;
  }
}

//Called by GalilController at start up once only.  
//Asyn does connection management for us, and gives callbacks at status change
void GalilController::connect(void)
{
  asynInterface *pasynInterface;	//To retrieve required asyn interfaces
  string address_string;		//Temporary address string used to setup communications
  int sync_connected;			//Is the synchronous communication socket connected according to asyn
  int async_connected = 1;		//Is the asynchronous communication connected according to asyn
  string address = address_;		//Convert address into string for easy inspection

  //Set default timeout at connect
  timeout_ = 1;

  //Construct the asyn port name that will be used for synchronous communication
  sprintf(syncPort_, "GALILSYNC%d", controller_number_);
  if (address.find("COM") == string::npos && address.find("ttyS") == string::npos)
     {
     //Open Synchronous ethernet connection
     //If no port number is specified in the provided address, append Telnet port and TCP directive
     //Otherwise just append the TCP directive
     if (address.find(":") == string::npos) {
        address_string = address + ":23 TCP";
     } else {
        address_string = address + " TCP";
     }

     //Connect to the device, we don't want end of string processing
     drvAsynIPPortConfigure(syncPort_, address_string.c_str(), epicsThreadPriorityMedium, 0, 1);

     if (try_async_)
        {
        //Create Asynchronous udp connection
        //Construct the asyn port name that will be used for asynchronous UDP communication
        sprintf(asyncPort_, "GALILASYNC%d", controller_number_);
        //Construct address to use for udp server
        //Ensure no port specifier
        if (address.find(":") != string::npos)
           address.erase(address.find_last_of(":"));
        //Add udp port and udp specifier
        address_string = address + ":60007 udp";
        //Connect to the device, we don't want end of string processing
        drvAsynIPPortConfigure(asyncPort_, address_string.c_str(), epicsThreadPriorityMax, 0, 1);
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
     drvAsynSerialPortConfigure(syncPort_, (char *)address_.c_str(), epicsThreadPriorityMax, 0, 1);
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

   //Set shutdown flag
   shuttingDown_ = true;

   //Destroy the poller for this GalilController.
   if (poller_ != NULL)
      {
      delete poller_;
      poller_ = NULL;
      }

   //Destroy the connector thread for this GalilController
   if (connector_ != NULL)
      {
      delete connector_; 
      connector_ = NULL;
      }

   //Threads are waiting on these events
   //Signal these events now shutDown flag is set
   epicsEventSignal(profileExecuteEvent_);
   epicsEventSignal(arrayUploadEvent_);
   //Destroy events
   epicsEventDestroy(connectEvent_);
   //Sleep to preempt this thread, and give time
   //for profile, and array upload thread to exit
   epicsThreadSleep(.002);
   //Destroy remaining events
   epicsEventDestroy(profileExecuteEvent_);
   epicsEventDestroy(arrayUploadEvent_);

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

      //Asyn exit handler will disconnect sync connection from here
      //We just print message to tell user Asyn epicsAtExit callback is running (next) and will close connection
      cout << "Disconnecting from " << model_ << " at " << address_ << endl;
      //Release the lock
      unlock();
      }

   //Free the memory where profileTimes_ is stored
   if (profileTimes_ != NULL)
      free(profileTimes_);
}

GalilController::~GalilController()
{
}

void GalilController::setParamDefaults(void)
{ 
  unsigned i;
  //Set defaults in Paramlist before connected
  //Set driver version
  setStringParam(GalilDriver_, driverVersion);
  //Pass address string provided by GalilCreateController to upper layers
  setStringParam(GalilAddress_, address_);
  //Set default model string
  setStringParam(GalilModel_, "Unknown");
  //SSI capable
  setIntegerParam(GalilSSICapable_, 0);
  //BISS capable
  setIntegerParam(GalilBISSCapable_, 0);
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
  //Output compare is off
  for (i = 0; i < 2; i++)
	setIntegerParam(i, GalilOutputCompareAxis_, 0);
  setStringParam(GalilSerialNum_, "");
  setStringParam(GalilEthAddr_, "");
  //Default all forward kinematics to null strings
  for (i = MAX_GALIL_CSAXES; i < MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
     setStringParam(i, GalilCSMotorForward_, "");

  //Amplifier faults are yet to be acquired
  for (i = 0; i < 2; i++) {
     //Set the AD amplifier overcurrent status
     setIntegerParam(i, GalilAmpOverCurrentStatus_, 0);
     //Set the AD amplifier overvoltage status
     setIntegerParam(i, GalilAmpOverVoltageStatus_, 0);
     //Set the AD amplifier overtemperature status
     setIntegerParam(i, GalilAmpOverTemperatureStatus_, 0);
     //Set the AD amplifier overtemperature status
     setIntegerParam(i, GalilAmpUnderVoltageStatus_, 0);
     //Set the AD amplifier ELO status
     setIntegerParam(i, GalilAmpELOStatus_, 0);
  }

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
  char RV[] = {0x12,0x16,0x0};   //Galil command string for model and firmware version query
  int status;
  double minUpdatePeriod;        //Min update period given model
  string mesg = "";              //Connected mesg
  unsigned i;                    //Looping

  //Flag connected as true
  connected_ = true;
  setIntegerParam(GalilCommunicationError_, 0);
  //Load model, and firmware query into cmd structure
  strcpy(cmd_, RV);
  //Query model, and firmware version
  sync_writeReadController();
  //store model, and firmware version in GalilController instance
  model_ = tsp(resp_);
  //Pass model string to ParamList
  setStringParam(GalilModel_, model_);
  //Determine if controller is dmc or rio
  rio_ = (model_.find("RIO") != string::npos) ? true : false;
  //Give connect message
  mesg = "Connected to " + model_ + " at " + address_;
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
     mesg = "Restricting UpdatePeriod to " + tsp(minUpdatePeriod, 0) + "ms minimum";
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

  //Determine if controller is BISS capable
  if (model_.find("BISS") != string::npos)
     setIntegerParam(GalilBISSCapable_, 1);
  else
     setIntegerParam(GalilBISSCapable_, 0);

  //Determine if controller is SSI capable
  if (model_.find("SSI") != string::npos)
     setIntegerParam(GalilSSICapable_, 1);
  else
     setIntegerParam(GalilSSICapable_, 0);

  if (model_.find("SER") != string::npos)
     {
     //Could be either SSI, or BISS, or both
     //Determine if controller is SSI capable
     strcpy(cmd_, "SIA=?");
     status = sync_writeReadController();
     if (numAxesMax_ > 4)
        {
        strcpy(cmd_, "SIE=?");
        status &= sync_writeReadController();
        }
     if (status == asynSuccess)
        setIntegerParam(GalilSSICapable_, 1);
     else
        setIntegerParam(GalilSSICapable_, 0);

     //Determine if controller is BISS capable
     strcpy(cmd_, "SSA=?");
     status = sync_writeReadController();
     if (numAxesMax_ > 4)
        {
        strcpy(cmd_, "SSE=?");
        status &= sync_writeReadController();
        }
     if (status == asynSuccess)
        setIntegerParam(GalilBISSCapable_, 1);
     else
        setIntegerParam(GalilBISSCapable_, 0);
   }

  //Determine if controller is Ethercat capable
  if (model_[3] == '5')
     setIntegerParam(GalilEtherCatCapable_, 1);
  else
     setIntegerParam(GalilEtherCatCapable_, 0);

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

  //Update installed amplifier information
  updateAmpInfo();

  //Determine number of threads supported
  //Safe default
  numThreads_ = 8;
  //Check for controllers that support < 8 threads
  //RIO
  numThreads_ = (rio_) ? 4 : numThreads_;
  //DMC3 range
  if ((model_[0] == 'D' && model_[3] == '3'))
     numThreads_ = 6;
  //DMC1 range
  numThreads_ = (model_[3] == '1')? 2 : numThreads_;

  //Stop all threads running on the controller
  for (i = 0; i < numThreads_; i++)
     {
     sprintf(cmd_, "HX%d",i);
     sync_writeReadController();
     }

  //Stop all moving motors
  for (i = 0; i < numAxesMax_; i++)
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
     //Deliver and start the code on controller
     GalilStartController(code_file_, burn_program_, thread_mask_);
     }

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
  unsigned axis;
  GalilAxis *pAxis;
  double pollDelay;

  getDoubleParam(GalilStatusPollDelay_, &pollDelay);
  fprintf(fp, "Galil motor driver %s, IP address=%s, numAxes=%d, Amp1 model=%d, Amp2 model=%d, poll delay=%f\n", 
    this->portName, address_.c_str(), numAxes_, ampModel_[0], ampModel_[1], pollDelay);
  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %c\n"
              "    ready=%s\n"
              "    encoder position=%f\n"
              "    motor position=%f\n",
              pAxis->axisName_,
              pAxis->axisReady_ ? "true" : "false",
              pAxis->encoder_position_, pAxis->motor_position_);
    }
  }

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

/** Returns true if any profile axis is moving
  * An axis is considered moving until all retries, backlash completed
  */
bool GalilController::anyMotorMoving()
{
  GalilAxis *pAxis;	//GalilAxis instance
  int moving = 0;	//Moving status
  int dmov;		//Axis motor record dmov
  int i;		//Looping

  //Look through profileAxes list
  //Return true if any moving
  for (i = 0; profileAxes_[i] != '\0'; i++) {
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

/** Returns true if all profile axis are moving
  * An axis is considered moving until all retries, backlash completed
  */
bool GalilController::allMotorsMoving()
{
  int moving;           //Moving status
  GalilAxis *pAxis;     //GalilAxis instance
  int dmov;             //Axis motor record dmov
  int i;                //Looping

  //Look through profileAxes list
  //Return false if any not moving
  for (i = 0; profileAxes_[i] != '\0'; i++) {
     //Retrieve the axis
     pAxis = getAxis(profileAxes_[i] - AASCII);
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
  string mesg;          //Profile execute message
  GalilAxis *pAxis;     //Real motor instance
  bool atStart = true;  //Are all motors in axes list moving or stopped without limit
  int moveMode;         //Move mode absolute or relative
  int moving;           //Axis moving status
  int dmov;             //Axis done moving
  int fwd, rev;         //Axis fwd, rev limit status
  int axisNo;           //Axis number
  double readback;      //Axis position readback in egu
  double start;         //Desired motor start position in egu
  double eres, mres;    //Encoder, and motor resolution
  double epos, mpos;    //Encoder, motor position
  double off;           //User offset
  int dir, dirm;        //Direction multiplier
  int ueip;             //Use encoder if present
  double rdbd;          //Motor record retry deadband
  int i;                //Looping

  //Look through motor list
  for (i = 0; profileAxes_[i] != '\0'; i++) {
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
     if ((!moving && dmov && (readback < (start - rdbd))) || (!moving && dmov && (readback > (start + rdbd))) ||
         (!moving && (rev || fwd))) {
        atStart = false;
        break;
     }
  }

  if (!atStart) {
     //Store message in paramList
     if (rev || fwd)
        mesg = "Profile motor " + string(1, pAxis->axisName_) + " hit limit whilst moving to profile start position"; 
     else
        mesg = "Profile motor " + string(1, pAxis->axisName_) + " at " + tsp(readback, 4) + " did not reach start " + tsp(start, 4) + " within retry deadband " + tsp(rdbd, 4);
     setStringParam(profileExecuteMessage_, mesg.c_str());
  }

  //Motors were all at profile start position
  return atStart;
}

/** setOutputCompare function.  For turning output compare on/off
  * \param[in] output compare to setup - 0 or 1 for output compare 1 and 2 */
asynStatus GalilController::setOutputCompare(int oc)
{
  string mesg;                  //Output compare message
  int ocaxis;                   //Output compare axis from paramList
  int axis;                     //Axis number derived from output compare axis in paramList
  int motor;                    //motor type read from controller
  int start, end;               //Looping
  double ocstart;               //Output compare start position from paramList
  double ocincr;                //Output compare incremental distance for repeat pulses from paramList
  int dir, dirm = 1;            //Motor record dir, and dirm direction multiplier based on motor record DIR field
  double off;                   //Motor record offset
  double eres;                  //mr eres
  int mainencoder, auxencoder;  //Main and aux encoder setting
  int encoder_setting;          //Overall encoder setting value
  bool mtr_encoder_ok = true;   //Motor and encoder setting status on selected axis (ie. ok or not)
  bool setup_ok = false;        //Overall setup status
  int comstatus = asynSuccess;	//Status of comms
  int paramstatus = asynSuccess;//Status of paramList gets
  int i;                        //Looping

  //Output compare not valid on some controllers
  //Output compare 1 valid for controllers > 0 axis
  //Output compare 2 valid for controllers > 4 axis
  if ((oc && numAxesMax_ <= 4) || (!oc && !numAxesMax_) || (!connected_))
     return asynSuccess;

  //Retrieve axis to use with output compare
  paramstatus = getIntegerParam(oc, GalilOutputCompareAxis_, &ocaxis);

  //Attempt turn on selected output compare
  if (ocaxis && !paramstatus) {
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
     if (encoder_setting != 0 && encoder_setting != 5 && encoder_setting != 10 && encoder_setting != 15 && !paramstatus) {
        //Main and auxillary encoder settings are not the same on requested axis, reject!
        mesg = "Output compare " + tsp(oc+1,0) + " failed, " + string(1, axis + AASCII) + " main and aux encoder settings need to be same";
        paramstatus = setStringParam(GalilOutputCompareMessage_, mesg.c_str());
        setIntegerParam(oc, GalilOutputCompareAxis_, 0);
        mtr_encoder_ok = false;
     }
     //Check motor settings
     if (abs(motor) != 1 && abs(motor) != 1.5) {
        //Motor type is not servo on requested axis, reject!
        mesg = "Output compare " + tsp(oc+1,0) + " failed, " + string(1, axis + AASCII) + " motor type must be servo";
        paramstatus = setStringParam(GalilOutputCompareMessage_, mesg.c_str());
        setIntegerParam(oc, GalilOutputCompareAxis_, 0);
        mtr_encoder_ok = false;
     }
     //If motor/encoder settings ok, no paramlist error, and no command error
     if (mtr_encoder_ok && !paramstatus && !comstatus) {
        //Passed motor configuration checks.  Motor is servo, and encoder setting is ok
        //Retrieve output compare start, and increment values
        paramstatus |= getDoubleParam(oc, GalilOutputCompareStart_, &ocstart);
        paramstatus |= getDoubleParam(oc, GalilOutputCompareIncr_, &ocincr);
        //Retrieve the axis encoder resolution
        paramstatus |= getDoubleParam(axis, GalilEncoderResolution_, &eres);
        //Retrieve axis offset, and dir
        paramstatus |= getIntegerParam(axis, GalilDirection_, &dir);
        paramstatus |= getDoubleParam(axis, GalilUserOffset_, &off);
        //Calculate direction multiplier
        dirm = (dir == 0) ? 1 : -1;
        //Convert start and increment to steps
        //Start is in user coordinates
        ocstart = ((ocstart - off)/eres) * dirm;
        //Increment is relative, so OFF isn't included
        ocincr = (ocincr/eres) * dirm;
        //Check start, and increment values
        //Note that -2147483647 - 1 avoids warning C4146 on microsoft compilers
        if (rint(ocincr) >= -65536 && rint(ocincr) <= 65535 &&
           rint(ocstart) >= -2147483647 - 1 && rint(ocstart) <= 2147483647 && !paramstatus) {			
           sprintf(cmd_, "OC%c=%.0lf,%.0lf", axis + AASCII, rint(ocstart), rint(ocincr));
           comstatus = sync_writeReadController();
           setup_ok = (!comstatus) ? true : false;
           if (setup_ok) {
              mesg = "Output compare " + tsp(oc+1,0) + " setup successfully";
              setStringParam(GalilOutputCompareMessage_, mesg.c_str());
           }
           else {
              //Reject motor setting if problem
              mesg = "Output compare " + tsp(oc+1,0) + " setup failed";
              setStringParam(GalilOutputCompareMessage_, mesg.c_str());
              setIntegerParam(oc, GalilOutputCompareAxis_, 0);
           }
        }
        else {
           //Reject motor setting if problem with start or increment
           mesg = "Output compare " + tsp(oc+1,0) + " failed due to start/increment out of range";
           setStringParam(GalilOutputCompareMessage_, mesg.c_str());
           setIntegerParam(oc, GalilOutputCompareAxis_, 0);
        }
     }
  }

  //Attempt turn off selected output compare
  if (!setup_ok && !paramstatus) {
     //Default parameters
     axis = 99;
     //Calculate loop start/end
     start = (!oc) ? 0 : 4;
     end = (!oc) ? 4 : 8;
	
     //Find a servo in correct bank either A-D, or bank E-H
     for (i = start; i < end; i++) {
        sprintf(cmd_, "MT%c=?", i + AASCII);
        comstatus = sync_writeReadController();
        motor = atoi(resp_);
        if (abs(motor) == 1 || abs(motor) == 1.5) {
           axis = i;
           break;
        }
     }
	
     if (axis != 99) {
        //A servo was found in the correct bank
        sprintf(cmd_, "OC%c=0,0", axis + AASCII);
        comstatus = sync_writeReadController();
        if (!ocaxis) {
           mesg = "Output compare " + tsp(oc+1,0) + " turned off";
           setStringParam(GalilOutputCompareMessage_, mesg.c_str());
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
 * Param [in] string mesg - Message if error occurs
 * @return motor driver status code.
 */
asynStatus GalilController::checkCSAxisProfiles(string &mesg)
{
  unsigned i, j;				//Looping
  GalilCSAxis *pCSAxes[MAX_GALIL_AXES] = {0};	//GalilCSAxis list
  int useCSAxis[MAX_GALIL_CSAXES] = {0};	//CSAxis useAxis flag for profile moves
  int useAxis[MAX_GALIL_AXES] = {0};		//Axis useAxis flag for profile moves
  int moveCSMode[MAX_GALIL_AXES] = {0};		//CSAxis moveMode flag for profile moves
  int moveMode[MAX_GALIL_AXES] = {0};		//Axis moveMode flag for profile moves
  string str1, str2;				//String searching
  size_t found;					//String searching

  //Collect all CSAxis instances, moveModes, and useAxes flags
  for (i = 0; i < MAX_GALIL_CSAXES; i++) {
     //Retrieve profileUseAxis_ from ParamList for CSAxis
     getIntegerParam(i + MAX_GALIL_AXES, profileUseAxis_, &useCSAxis[i]);
     //Retrieve GalilProfileMoveMode_ from ParamList for CSAxis
     getIntegerParam(i + MAX_GALIL_AXES, GalilProfileMoveMode_, &moveCSMode[i]);
     //Retrieve GalilCSAxis instance
     pCSAxes[i] = getCSAxis(i + MAX_GALIL_AXES);
  }

  //Collect all Axis moveModes, and useAxes flags
  for (i = 0; i < MAX_GALIL_AXES; i++) {
     //Retrieve profileUseAxis_ from ParamList for Axis
     getIntegerParam(i, profileUseAxis_, &useAxis[i]);
     //Retrieve GalilProfileMoveMode_ from ParamList for Axis
     getIntegerParam(i, GalilProfileMoveMode_, &moveMode[i]);
  }

  //Loop thru included CSAXis
  //Look for setup problems
  for (i = 0; i < MAX_GALIL_CSAXES; i++) {
     //Decide to process this axis, or skip
     if (!useCSAxis[i] || !pCSAxes[i]) continue;
     //Pass reverse axis list to std::string type for processing
     str1 = pCSAxes[i]->revaxes_;
     //Loop thru all motors in this CSAxis
     //Ensure useAxis set true on all motors
     //Ensure motor moveMode matches CSAxis
     for (j = 0; j < strlen(str1.c_str()); j++) {
        //Ensure useAxis set true
        if (!useAxis[pCSAxes[i]->revaxes_[j] - AASCII]) {
           //Update the message
           mesg = string(1, pCSAxes[i]->revaxes_[j]) + " axis UseAxis flag is set to no, it should be set yes instead";
           return asynError;
        }
        //Ensure Axis movemode matches the CSAxis
        if (moveMode[pCSAxes[i]->revaxes_[j] - AASCII] != moveCSMode[i]) {
           //Update the message
           mesg = string(1, pCSAxes[i]->revaxes_[j]) + " axis moveMode does not match CSAxis " + string(1, pCSAxes[i]->axisName_);
           return asynError;
        }
     }
     //Check other included CSAxis revaxes against this one
     //If any CSAxis in profile share a motor, then fail
     for (j = 0; j < MAX_GALIL_CSAXES; j++) {
        //Decide to process this axis, or skip
        if (i == j) continue;
        if (!useCSAxis[j] || !pCSAxes[j]) continue;
        str2 = pCSAxes[j]->revaxes_;
        found = str1.find_first_of(str2);
        if (found != string::npos) {
           //Update the message
           mesg = tsp(str1[found]) + " axis cannot be shared.  Review axis included in profile";
           return asynError;
        }
     }
  } //For

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
  const char *functionName = "buildProfileFile";
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
  string mesg;  			//Profile build message
  int useAxis[MAX_GALIL_AXES];		//Use axis flag for profile moves
  int moveMode[MAX_GALIL_AXES];		//Move mode absolute or relative
  string moves = "";			//Segment move command assembled for controller
  string axes = "";			//Motors involved in profile move
  string startp = "";//Profile start positions written to file
  char fileName[MAX_FILENAME_LEN];	//Filename to write profile data to
  FILE *profFile;			//File handle for above file
  bool buildOK=true;			//Was the trajectory built successfully
  double mres;				//Motor resolution
  double temp_time;			//Used for timebase calculations in PVT mode
  int status;				//Return status

  //Check CSAxis profiles
  //Ensure motors are not shared
  //Ensure useAxis, and moveMode between CSAxes, and
  //member reverse (real) motors are consistent
  status = checkCSAxisProfiles(mesg);

  //Transform CSAxis profiles to axis profiles
  if (!status)
     status |= transformCSAxisProfiles();

  //Retrieve required attributes from ParamList
  status |= getStringParam(GalilProfileFile_, (int)sizeof(fileName), fileName);
  status |= getIntegerParam(profileNumPoints_, &nPoints);
  status |= getIntegerParam(GalilPVTCapable_, &pvtcapable);
  status |= getIntegerParam(GalilProfileType_, &proftype);

  //Check provided fileName
  if (!status && !abs(strcmp(fileName, ""))) {
     mesg = string(functionName) + ": Bad trajectory file name";
     status = asynError;
  }

  //Build profile file
  if (!status) {
     //No error so far
     //Create the profile file
     profFile =  fopen(fileName, "wt");

     //Write profile type
     if (proftype)
        fprintf(profFile,"PVT\n");
     else
        fprintf(profFile,"LINEAR\n");

     //Zero variables, construct axes, start position, and maxVelocity lists 
     for (i = 0; i < MAX_GALIL_AXES; i++) {
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
        axes += (char)(i + AASCII);
        //Construct start positions list
        startp += tsp(rint(pAxis->profilePositions_[0]), 0) + ",";
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
     } //For

     //Write axes list
     fprintf(profFile,"%s\n", axes.c_str());

     //Write start positions list
     fprintf(profFile,"%s\n", startp.c_str());

     //Determine number of motors in profile move
     num_motors = (int)axes.size();

     //Calculate motor segment velocities from profile positions, and common time base
     for (i = 0; i < nPoints; i++) {
        //No controller moves assembled yet for this segment
        moves.clear();
        //velocity for this segment
        vectorVelocity = 0.0;
        //motors with zero moves for this segment
        zm_count = 0;
        //Calculate motor incremental move distance, and velocity
        for (j = 0; j < MAX_GALIL_AXES; j++) {
           //Retrieve GalilAxis
           pAxis = getAxis(j);
           //Decide to process this axis, or skip
           if (!useAxis[j] || !pAxis) {
	      //Linear mode, add axis relative move separator character ',' as needed
              if ((j < MAX_GALIL_AXES - 1) && !proftype)
                 moves += ',';
              //Skip the rest, this axis is not in the profile move
              continue;
           }
           if (i == 0) {
              //First segment incremental move distance
              firstmove[j] = pAxis->profilePositions_[i];
              //Velocity set to 0 for first increment
              incmove = velocity[j] = 0.0;
           }
           else {
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
              if (fabs(aerr[j]) > 1) {
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
           if (fabs(velocity[j]) > fabs(maxAllowedVelocity[j])) {
              mesg = "Seg " + tsp(i) + ": Velocity too high motor " + string(1, pAxis->axisName_) + " ";
              mesg += tsp(fabs(velocity[j]*mres), 2) + " > " + tsp(maxAllowedVelocity[j]*mres, 2);
              mesg += ", increase time, check profile";
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
              pAxis->profilePositions_[i] > pAxis->highLimit_) {
              //Only if move mode = Absolute
              if (moveMode[j]) {
                 mesg =  "Axis " + string(1, pAxis->axisName_) + " position beyond soft limits in segment ";
                 mesg += tsp(i);
                 buildOK = false;
              }
           }

           if (!proftype) {
              //Sum segment vector velocity for non zero move increments
              if (rint(incmove) != 0) {
                 //Calculate linear mode velocity
                 //Add this motors' contribution to segment vector velocity
                 vectorVelocity += pow(velocity[j]/timeMultiplier_, 2);
              }
              //Store motor incremental move distance for this segment
              moves += tsp(rint(incmove),0);
              //Add axis relative move separator character ',' as needed
              if (j < MAX_GALIL_AXES - 1)
                 moves += ',';
           }
           else {
              //PVT mode
              moves += tsp(profileTimes_[i]) + " " + tsp(rint(incmove),0) + ",";
              moves += tsp(velocity[j],0) + "," + tsp(rint(profileTimes_[i]*1000.0*timeMultiplier_),0) + '\n';
              //Check timebase is multiple of 2ms
              temp_time = profileTimes_[i] * 1000.0;
              //PVT has 2 sample minimum
              if ((int)(rint(temp_time)) % 2 != 0) {
                 mesg = "Profile time base must be multiple of 2ms in PVT mode";
                 buildOK = false;
              }
           }
           //Detect zero moves in this segment
           zm_count =  (rint(incmove) == 0) ? zm_count+1 : zm_count;
           //Store current segment velocity for this motor
           velocityPrev[j] = velocity[j];
        } //For axis

        //Check for zero move segments
        if (zm_count == num_motors && i != 0) {
           mesg = "Seg " + tsp(i) + ": Vector zero move distance, reduce time, add motors, and check profile";
           if (!num_motors)
              mesg = "No motors in profile, add motors";
           buildOK = false;
        }

        if (!proftype) {
           //Linear mode
           //Determine vector velocity for this segment
           vectorVelocity = sqrt(vectorVelocity);
           //Check for segment too short error
           if (rint(vectorVelocity) == 0 && i != 0) {
              mesg = "Seg " + tsp(i) + ": Vector velocity zero, reduce time, add motors, and check profile";
              buildOK = false;
           }
           //Trim trailing ',' characters from moves string
           moves.erase(moves.find_last_of("0123456789")+1);
           //Add segment velocity
           moves += "<" + tsp(rint(vectorVelocity),0);
        }

        //Add second segment and above
        //First segment is the "relative" offset or start position for the profile
        //This is done to prevent jumps in the motor
        if (i > 0) {
           //Write the segment command to profile file
           if (proftype) //PVT
              fprintf(profFile,"%s", moves.c_str());
           else	//Linear
              fprintf(profFile,"%lf %s\n", profileTimes_[i], moves.c_str());
        }
     } //For nPoints
  //Profile written to file now close the file
  fclose(profFile);
  }
  else //Bad status, so build failed
     buildOK = false;

  //Check, and if neccessary restore GalilAxis original profile
  //data after CSAxis profiles have been built
  for (i = 0; i < MAX_GALIL_AXES; i++) {
     //Retrieve GalilAxis
     pAxis = getAxis(i);
     //Decide to process this axis, or skip
     if (!pAxis) continue;
     pAxis->restoreProfileData();
  }

  //Build failed.  
  if (!buildOK) {
     //Delete profile file because its not valid
     remove(fileName);
     //Update build mesg
     setStringParam(profileBuildMessage_, mesg.c_str());
     return asynError;
  }
  else {
     //Update profile ParamList attributes if buildOK
     for (j = 0; j < MAX_GALIL_AXES; j++) {
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
  static const char *functionName = "buildProfile";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);
            
  //Call the base class method which will build the time array if needed
  asynMotorController::buildProfile();

  //Update profile build status
  setStringParam(profileBuildMessage_, "");
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
    if (shuttingDown_)
       break;//Exit the thread
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
    if (shuttingDown_)
       break; //Exit the thread
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
   int i;                       //Looping
   int autoonoff;               //Auto on/off setting
   int axisNo;                  //Axis number
   GalilAxis *pAxis;            //GalilAxis
   double ondelay;              //GalilAxis auto on delay
   double largest_ondelay = 0.0;//Largest auto on delay in motor list
   int motoroff;                //Motor amplifier off status
   int autobrake;               //Brake auto disable/enable setting
   int brakeport;               //Brake digital out port
   int brakeoff;                //Motor brake off status
   string cmd = "";             //Controller command

   //Iterate thru motor list and execute AutoOn for each axis
   for (i = 0; axes[i] != '\0'; i++) {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve axis instance
      pAxis = getAxis(axisNo);
      //Execute AutoOnBrakeOff for this axis
      if (!pAxis) continue;
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
      //Execute auto motor on if feature enabled
      if (motoroff && autoonoff) {
            //Add command to turn on this axis amplifier
            if (!cmd.empty()) cmd += ";";
            cmd += "SH" + string(1, pAxis->axisName_);
      }

      //Query brake status direct from controller
      sprintf(cmd_, "MG @OUT[%d]", brakeport);
      sync_writeReadController();
      brakeoff = atoi(resp_);
      //Execute auto brake off if feature enabled
      if (!brakeoff && brakeport > 0 && autobrake) {
            //Add command to turn off this axis brake
            if (!cmd.empty()) cmd += ";";
            cmd += "SB " + tsp(brakeport);
      }
   }

   //Test if any work required
   if (!cmd.empty()) {
      //Write command constructed
      strcpy(cmd_, cmd.c_str());
      sync_writeReadController();
      //Wait the longest auto on time found in list
      if (largest_ondelay > 0.035) {
         //Delay long enough to unlock, and allow other threads to run
         unlock();
         //Wait auto on delay
         epicsThreadSleep(largest_ondelay);
         //Get the lock
         lock();
      }
      else
         epicsThreadSleep(largest_ondelay);
   }
}

/* For profile moves.  Convenience function to move motors to start or stop them moving to start
*/
asynStatus GalilController::motorsToProfileStartPosition(double startp[], bool move = true)
{
  const char *functionName = "motorsToProfileStartPosition";
  GalilAxis *pAxis;               //GalilAxis
  int i;                          //Axis looping
  int j;                          //Ensure while loop doesn't get trapped
  int axisNo;                     //Axis number
  int moveMode;                   //Move mode absolute or relative
  int deferredMode;               //Backup of current deferredMode
  string mesg = "";               //Profile execute message
  char axis = ' ';                //Axis that caused fault
  int status = asynSuccess;       //Return status

  if (move) {
     //Check all axis settings
     status = checkAllSettings(functionName, profileAxes_, startp, &axis);
     //Check for problems
     if (status) {
        //An axis has a problem, abort profile
        //Update profile execute message
        mesg = "Profile axis " + string(1, axis) + " is not ready, check controller message";
     }
     else {
        //Update profile execute status
        mesg = "Moving motors to start position and buffering profile data...";
        //Set deferred moves true
        setDeferredMoves(true);
        //Retrieve deferred moves mode
        getIntegerParam(GalilDeferredMode_, &deferredMode);
        //We must use sync start only mode
        //To avoid interferring with profile download
        setIntegerParam(GalilDeferredMode_, 0);
     }
     //Set profile execute message
     setStringParam(profileExecuteMessage_, mesg.c_str());
     callParamCallbacks();
  } //move

  //If move mode absolute then set motor setpoints to profile start position
  //or stop motors moving to start position
  if ((move && !status) || !move) {
     for (i = 0; profileAxes_[i] != '\0'; i++) {
        //Determine the axis number
        axisNo = profileAxes_[i] - AASCII;
        //Retrieve GalilProfileMoveMode_ from ParamList
        getIntegerParam(axisNo, GalilProfileMoveMode_, &moveMode);
        //If moveMode = Relative skip move to start
        if (!moveMode && move) continue;
        //Retrieve axis instance
        pAxis = getAxis(axisNo);
        //Skip axis if not instantiated
        if (!pAxis) continue;
        //If moveMode = Absolute, then set motor setpoints to profile start position
        if (move) {
           //Set motor setpoints, but dont do the move (movesDeferred = true)
           if (!pAxis->moveThruMotorRecord(startp[i])) {
              //Requested move equal or larger than 1 motor step, move success
              //Count number of preempts
              j = 0;
              //Unlock mutex so GalilAxis::move is called
              //Also give chance for sync poller to get the lock
              unlock();
              //Preempt thread till GalilAxis::move is called
              while (!pAxis->deferredMove_) {
                 epicsThreadSleep(.001);
                 //Give up after 200 ms/preempts
                 if (j++ > 200) {
                    break;
                 }
              }
              //Check for error
              if (!pAxis->deferredMove_ && pAxis->moveThruRecord_) {
                 //GalilAxis::move wasn't called
                 //Set flag false
                 pAxis->moveThruRecord_ = false;
                 //Disable further writes to axis motor record from driver
                 setIntegerParam(pAxis->axisNo_, GalilMotorSetValEnable_, 0);
              }
              //Done, move on to next motor
              lock();
           }
        }
        else {
           //Set flag indicating axis motor record will be stopped by driver
           pAxis->stopInternal_ = true;
           //Prevent the axis motor record issuing backlash, retries
           pAxis->stopMotorRecord();
        }
     }//For
   }

  if (move && !status) {
     //Restore previous deferredMode
     setIntegerParam(GalilDeferredMode_, deferredMode);
     //Release deferred moves, start all motors together
     setDeferredMoves(false);
  }
  
  if (!move) {
     //Stop all profile motors
     abortProfile();
     //Release lock
     unlock();
     //Wait for motion to complete
     while (anyMotorMoving())
        epicsThreadSleep(.001);
     //Obtain the lock
     lock();
  }

  //Return status
  return (asynStatus)status;
}

/* Convenience function to begin profile
   Called after filling the buffer
*/
asynStatus GalilController::beginProfileMotion(int coordsys, char coordName)
{
   const char *functionName = "beginProfileMotion";
   char axis;
   string mesg = "";
   int status = asynSuccess;

   //Check profile axes are ready to go
   status = beginCheck(functionName, profileAxes_, &axis);

   //Check for error
   if (!status) {
      //Profile axes are ready to go, start profile
      if (!profileType_)
         status |= beginLinearGroupMotion(coordName, profileAxes_);
      else
         status |= beginPVTProfileMotion();
   }
   else {//A profile axis wasn't ready
      mesg = "Profile axis " + string(1, axis) + " is not ready, check controller message";
   }

   //Set message
   if (status) {
      //Start fail
      if (mesg.empty())
         mesg = "Profile start failed...";
      //Store message in ParamList
      setStringParam(profileExecuteMessage_, mesg);
   }
   else {
      //Start success
      setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_EXECUTING);
      mesg = "Profile executing...";
      //Store message in ParamList
      setStringParam(profileExecuteMessage_, mesg);
   }

   //Post message
   callParamCallbacks();
   //Return status
   return (asynStatus)status;
}

/* Convenience function to begin PVT group
   Called after filling the PVT buffer
*/
asynStatus GalilController::beginPVTProfileMotion()
{
  unsigned i;				//Looping
  const char *functionName = "beginPVTProfileMotion";
  string mesg;              //Controller mesg

  //Execute motor auto on and brake off function
  executeAutoOnBrakeOff(profileAxes_);

  //Execute motor record prem
  executePrem(profileAxes_);

  //Set accel/decel for profileAxes_
  for (i = 0; profileAxes_[i] != '\0'; i++) {
     //Set acceleration/deceleration
     sprintf(cmd_, "AC%c=%ld;DC%c=%ld", profileAxes_[i], maxAcceleration_, profileAxes_[i], maxAcceleration_);
     sync_writeReadController();
  }

  //Begin the move
  //Get time when attempt motor begin
  sprintf(cmd_, "BT %s", profileAxes_);
  if (sync_writeReadController() == asynSuccess) {
     //Wait till moving true on all axes delivered to mr
     //or timeout occurs
     axesEventMonitor(profileAxes_);
  }
  else {
     //Controller gave error at begin
     mesg = string(functionName) + ": Begin failure " + string(profileAxes_);
     //Set controller error mesg monitor
     setCtrlError(mesg);
     return asynError;
  }

  //Return success
  return asynSuccess;
}

/* Convenience function to begin linear group using specified coordinate system
   Called after filling the linear buffer
*/
asynStatus GalilController::beginLinearGroupMotion(char coordName, const char *axes)
{
  const char *functionName = "beginLinearGroupMotion";
  string mesg;          	//Controller mesg

  //Execute motor auto on and brake off function
  executeAutoOnBrakeOff(axes);

  //Execute motor record prem
  executePrem(axes);

  //Begin the move
  sprintf(cmd_, "BG %c", coordName);
  if (sync_writeReadController() == asynSuccess) {
     //Wait till moving true on all axes delivered to mr
     //or timeout occurs
     axesEventMonitor(axes);
  }
  else {
     //Controller gave error at begin
     mesg = string(functionName) + ": Begin failure " + string(axes);
     //Set controller error mesg monitor
     setCtrlError(mesg);
     return asynError;
  }

  //Return success
  return asynSuccess;
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
  unsigned i;		    			//Looping
  int status;					//Return value
  string mesg;      				//Profile execute message
  char fileName[MAX_FILENAME_LEN];	//Filename to read profile data from
  char profileType[MAX_GALIL_STRING_SIZE];	//String read from trajectory file that represents profile type (ie. LINEAR or PVT)
  int pvtcapable;				//Controller PVT capable status
  char axis;					//Axis

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
  status = checkCSAxisProfiles(mesg);
  
  //Open the profile file
  if (!status) {
     //Retrieve trajectory filename from ParamList
     getStringParam(GalilProfileFile_, (int)sizeof(fileName), fileName);
     //Open trajectory file
     if ((*profFile =  fopen(fileName, "rt")) == NULL) {
        //Assemble error message
        mesg = string(functionName) + ": Can't open trajectory file";
        status = asynError;
     }
  }

  if (!status) {
     //Read file header, and grab profile type
     fscanf(*profFile, "%s\n", profileType);
     //Determine profile type LINEAR or PVT type
     profileType_ = strcmp(profileType, "PVT") == 0 ? 1 : 0;
     //Determine which motors are involved
     fscanf(*profFile, "%s\n", profileAxes_);
     //Read profile start position for all profile axis
     for (i = 0; profileAxes_[i] != '\0'; i++) {
        //Read profile start positions from file
        fscanf(*profFile, "%lf,", &startp[i]);
     }
     //Move file pointer down a line
     fscanf(*profFile, "\n");

     //Check profile axes settings
     status = checkAllSettings(functionName, profileAxes_, startp, &axis);
     if (status) {
        //An axis has a problem, abort profile
        //Update profile execute message
        mesg = "Profile axis " + string(1, axis) + " is not ready, check controller message";
     }
  }

  //Check profileType_ against controller capability
  if (profileType_ && !pvtcapable && !status) {
     mesg = string(functionName) + ": Controller is not PVT capable";
     status = asynError;
  }

  //Setup profile
  if (!profileType_ && !status) {
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
  else if (!status) {
     //PVT profile
     for (i = 0; profileAxes_[i] != '\0'; i++) {
        //Clear any segments in the pvt buffer(s)
        sprintf(cmd_, "PV%c=0,0,-1", profileAxes_[i]);
        sync_writeReadController();
     }
  }

  //If any error, and mesg is not empty
  if (status && mesg.size() > 0) {
     //Set profile execute message
     setStringParam(profileExecuteMessage_, mesg);
  }

  //Return result
  return (asynStatus)status;
}

void GalilController::profileGetSegsMoving(int profStarted, int coordsys, int *moving, int *segprocessed)
{
   //Profile moving status
   *moving = anyMotorMoving();

   if (profileType_) {
      //PVT profile
      //Segments processed
      if (profStarted) {
         //Use first axis as measure of segs processed
         sprintf(cmd_, "MG _BT%c", profileAxes_[0]);
         sync_writeReadController();
         *segprocessed = atoi(resp_);
         //PVT end takes 1 segment, compensate in final segment count
         if (!*moving)
            *segprocessed -= 1;
      }
   }
   else {
      //Linear profile
      //Segments processed
      getIntegerParam(coordsys, GalilCoordSysSegments_, segprocessed);
   }

   //Update profile current point in ParamList
   setIntegerParam(profileCurrentPoint_, *segprocessed);
   callParamCallbacks();
}

/* Function to run trajectory.  It runs in a dedicated thread, so it's OK to block.
 * It needs to lock and unlock when it accesses class data. */ 
asynStatus GalilController::runProfileFile()
{
  int segsent;				//Segments loaded to controller so far
  char moves[MAX_GALIL_STRING_SIZE - 4];//Segment move command assembled for controller
                                        //Longest command prefix is 4 (eg. PVA=)
  string mesg= "";			//Profile execute message
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
  asynStatus status = asynSuccess;	//Final return value
  asynStatus prepResult = asynSuccess;	//Profile prep result

  //Called without lock, and we need it to call sync_writeReadController
  lock();

  //Clear controller error
  setCtrlError("");

  //Update profile execute state and status
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_MOVE_START);
  setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_UNDEFINED);
  callParamCallbacks();

  //prepare the profile
  //Retrieve profile file pointer, profile type, coordsys (linear only), coordName (linear only)
  //Retrieve profile axes list, and start positions from file
  prepResult = prepRunProfile(&profFile, &coordsys, &coordName, startp);

  if (!prepResult) {
     //Profile preparation okay so far
     //Move motors to start position
     prepResult = motorsToProfileStartPosition(startp);
  }

  //Profile preparation done
  //Release the lock
  unlock();

  //Check for profile preparation error
  if (!prepResult) {
     //All okay
     //No segments sent to controller just yet
     segsent = 0;
     //Number of segments processed
     segprocessed = 0;
     //Default maximum number of segments
     maxsegs = MAX_LINEAR_SEGMENTS;
     //Initialize local variables for PVT profile
     if (profileType_ && !status) {
        //Default maximum number of segments
        maxsegs = MAX_PVT_SEGMENTS;
        //Initialize PVT variables
        i = 0;
        nmotors = (unsigned)strlen(profileAxes_);
     }
     //Default number of segments to buffer before start
     startsegs = maxsegs;
  }
  else {
     //Profile preparation failed
     //Flag error
     status = asynError;
  }

  //Execute the profile
  //Loop till file downloaded to buffer, or error, or abort
  //Note Linear is 1 loop per segment
  //PVT is nmotor loops per segment
  while (retval != EOF && !status && !profileAbort_) {
     if (bufferNext) {
        //Read the segment
        retval = fscanf(profFile, "%lf %s\n", &profileTime, moves);
        //Store profile time
        profileTimes.push_back(profileTime);
     }

     lock();
     //Process segment if didnt hit EOF
     if (retval != EOF) {
        //Retrieve moving status, and segments processed
        profileGetSegsMoving(profStarted, coordsys, &pmoving, &segprocessed);

        //Case where profile has started, but then stopped
        if ((profStarted && !pmoving)) {
           unlock();
           //Give other threads a chance to get the lock
           epicsThreadSleep(0.001);
           break;	//break from loop
        }

        //Check if motors arrived at start position
        if (!profStarted && !atStart) {
           //Abort if motor stopped and not at start position, or limit
           if (!allMotorsMoving())
              status = motorsAtStart(startp) ? asynSuccess : asynError;
           if (!anyMotorMoving()) {
              status = motorsAtStart(startp) ? asynSuccess : asynError;
              atStart = (status) ? false : true;
           }
        }

        //Buffer next segment if no error, user hasnt pressed abort, there is buffer space
        if (!status && !profileAbort_ && bufferNext) {
           //Proceed to send next segment to controller
           if (!profileType_)	//LINEAR
              sprintf(cmd_, "LI %s", moves);
           else		//PVT
              sprintf(cmd_, "PV%c=%s", profileAxes_[i], moves);
           status = sync_writeReadController();
           if (status) {
              unlock();
              epicsThreadSleep(.2);
              lock();
              abortProfile();
              mesg = "Error downloading segment " + tsp(segsent + 1);
           }
           else	{
              //Increment segments sent to controller
              if (!profileType_) //Linear segment
                 segsent++;
              else {
                 //PVT profile
                 //Increment PVT axis
                 i++;
                 //Has a complete segment been sent?
                 if (i >= nmotors) {
                    //Full segment has been sent
                    segsent++;
                    //Again back to first PVT axis
                    i = 0;
                 }
              }
           }
        }

        //Ensure segs are being sent faster than can be processed by controller
        if (((segsent - segprocessed) <= 2) && profStarted && !status && !profileAbort_) {
           abortProfile();
           unlock();
           epicsThreadSleep(.2);
           lock();
           mesg = "Profile time base too fast";
           //break loop
           status = asynError;
        }

        //Check profile start condition
        if ((segsent - segprocessed) >= startsegs && !status && !profileAbort_) {
           //Buffered startsegs number of segments
           //Case where motors were moving to start position, and now complete, profile is not started.
           if (!profStarted && atStart && !status && !profileAbort_) {
              //Start the profile
              status = beginProfileMotion(coordsys, coordName);
              profStarted = (status) ? false : true;
           }
        }

        //Default bufferNext true before checking buffer
        bufferNext = true;
        //Check buffer, and abort status
        if ((segsent - segprocessed) >= maxsegs && !status && !profileAbort_) {
           //Segment buffer is full, and user has not pressed abort

           //Dont buffer next segment	
           //Give time for motors to arrive at start, or
           //Give time for controller to process a segment
           if ((anyMotorMoving() && !profStarted) || profStarted) {
              unlock();
              //Sleep for time equal to currently processing segment
              epicsThreadSleep(profileTimes[segprocessed]);
              bufferNext = false;
              lock();
           }
        }
     }

     //Release the lock
     unlock();
     //Give other threads a chance to get the lock
     epicsThreadSleep(0.001);
  } //While

  //Obtain the lock
  lock();

  if (!prepResult) {
     //Profile setup was okay
     //Finished sending segments to controller
     if (!profileType_) {
        //End linear interpolation mode
        strcpy(cmd_, "LE");
        sync_writeReadController();
     }
     else {
        //End PVT mode
        for (i = 0; profileAxes_[i] != '\0'; i++) {
           sprintf(cmd_, "PV%c=0,0,0", profileAxes_[i]);
           sync_writeReadController();
        }
     }
  }

  //Check if motors still moving to start position
  if (!profStarted && !status && !profileAbort_) {
     //Pause till motors stop
     while (anyMotorMoving()) {
        unlock();
        epicsThreadSleep(.01);
        lock();
     }
	
     //Start short profiles that fit entirely in the controller buffer <= MAX_SEGMENTS
     //If motors at start position, begin profile
     if (motorsAtStart(startp)) {
        status = beginProfileMotion(coordsys, coordName);
        profStarted = (status) ? false : true;
     }
  }

  //Profile not started, and motors still moving, stop them
  if (!profStarted && !prepResult && anyMotorMoving())
     motorsToProfileStartPosition(startp, false);  //Stop motors moving to start position

  //Finish up
  if (!status) {
     if (profStarted) {
        pmoving = 1;
        //Loop until stopped, or aborted
        while (pmoving) {
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
        mesg = "Profile completed successfully";
     else {
        //Not all segments were processed
        status = asynError;
        if (profileAbort_)
           mesg = "Profile stopped by user";
        else
           mesg = "Profile stopped by limit switch, soft limit, or other motor/encoder problem";
     }
  }

  //Set profile execute mesg
  if (mesg.size() > 0)
     setStringParam(profileExecuteMessage_, mesg);

  //Close the trajectory file
  if (profFile != NULL)
     fclose(profFile);

  //Release the lock
  unlock();

  //Return result
  return status;
}

asynStatus GalilController::runProfile()
{
  int status;				//asynStatus
  static const char *functionName = "runProfile";

  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: entry\n",
            driverName, functionName);

  //run the profile file
  status = runProfileFile();
  
  //Called without lock, obtain it
  lock();

  //Update profile execute state
  setIntegerParam(profileExecuteState_, PROFILE_EXECUTE_DONE);

  //Update profile execute status
  if (status)
	setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_FAILURE);
  else
	setIntegerParam(profileExecuteStatus_, PROFILE_STATUS_SUCCESS);
  callParamCallbacks();

  //Complete, release lock
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
asynStatus GalilController::beginSyncStartStopMove(int coordsys, const char *axes, const char *moves, double acceleration, double velocity)
{
  const char *functionName = "beginSyncStartStopMove";
  char coordName;		//Coordinate system name
  asynStatus status;		//Result
  string mesg;			//Controller error mesg if begin fail
  string cmd = "";		//Command string

  //Selected coordinate system name
  coordName = (coordsys == 0 ) ? 'S' : 'T';

  //Set the specified coordsys on controller
  cmd = "CA " + tsp(coordName);

  //Clear any segments in the coordsys buffer
  cmd += ";CS " + tsp(coordName);

  //Update coordinate system motor list at record layer
  setStringParam(coordsys, GalilCoordSysMotors_, axes);

  //Set linear interpolation mode and include motor list provided
  cmd += ";LM " + tsp(axes);

  //Set vector acceleration/decceleration
  cmd += ";VA" + tsp(coordName) + "=" + tsp(acceleration, 0);
  cmd += ";VD" + tsp(coordName) + "=" + tsp(acceleration, 0);

  //Set vector velocity
  cmd += ";VS" + tsp(coordName) + "=" + tsp(velocity, 0);

  //Write assembled command
  strcpy(cmd_, cmd.c_str());
  cmd.clear();
  sync_writeReadController();
 
  //Specify 1 segment
  cmd += "LI " + tsp(moves);

  //End linear mode
  cmd += ";LE";
  strcpy(cmd_, cmd.c_str());
  sync_writeReadController();

  //move the coordinate system
  status = beginLinearGroupMotion(coordName, axes);

  //Check for error
  if (status) {
     mesg = string(functionName) + ": Begin failure coordsys " + string(1, coordName);
     //Set controller error mesg monitor
     setCtrlError(mesg);
     status = asynError;
  }

  //Return status
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
  string axes = "";			//Constructed list of axis in the coordinate system
  string moves = "";			//Constructed comma list of axis relative moves
  double vectorAcceleration;		//Coordinate system acceleration
  double vectorVelocity;		//Coordinate system velocity
  bool csokay;				//Coordinate system is okay

  //Loop through coordinate systems, looking for work to perform
  for (coordsys = 0; coordsys < COORDINATE_SYSTEMS; coordsys++) {
     //No work found yet in this coordsys
     axes.clear();
     moves.clear();
     //velocity for this segment
     vectorVelocity = 0.0;
     vectorAcceleration = 0.0;
     //Default csokay flag
     csokay = true;

     //Loop through the axis looking for deferredMoves in this cs
     //Check axis okay, if problem set flag to later reject entire cs move
     //Construct axis list, and move command string
     for (axis = 0; axis < MAX_GALIL_AXES; axis++) {
        //Retrieve the axis
        pAxis = getAxis(axis);
        //Process or skip
        if (!pAxis) continue;
        //Look for deferred move
        if (pAxis->deferredCoordsys_ == coordsys && pAxis->deferredMove_ && pAxis->deferredMode_) {
           //Deferred move found in this coordsys
           //Ensure axis is good to go
           if (pAxis->beginCheck(functionName, pAxis->axisName_, pAxis->deferredVelocity_, false)) {
              //Axis has problem, set flag to reject all moves in this coordinate system
              csokay = false;
           }
           //Store axis in coordinate system axes list
           axes += pAxis->axisName_;
           //Store axis relative move
           moves += tsp(pAxis->deferredPosition_, 0);
           //Sum vector acceleration, velocity for non zero move increments
           if (rint(pAxis->deferredPosition_) != 0) {
              //Add this motors' contribution to vector acceleration for this segment
              vectorAcceleration += pow(pAxis->deferredAcceleration_, 2);
              //Add this motors' contribution to vector velocity for this segment
              vectorVelocity += pow(pAxis->deferredVelocity_, 2);
           }
        }
        //Add axis relative move separator character ',' as needed
        if (axis < MAX_GALIL_AXES - 1)
           moves += ',';
     } //For (axis)

     if (csokay && !axes.empty()) {
        //Coordinate system okay, and axes list not empty
        //Calculate final vectorVelocity and vectorAcceleration
        vectorVelocity = sqrt(vectorVelocity);
        vectorAcceleration = sqrt(vectorAcceleration);
        vectorVelocity = lrint(vectorVelocity/2.0) * 2;
        vectorAcceleration = lrint(vectorAcceleration/1024.0) * 1024;
        //Start the move
        beginSyncStartStopMove(coordsys, axes.c_str(), moves.c_str(), vectorAcceleration, vectorVelocity);
     }

     //Turn off deferredMove_ for these axes
     clearDeferredMoves(axes.c_str());

  } //For (coordinate system)

  return asynSuccess;
}

/**
 * Start a group of motors in independent mode simultanously
 * @param [in] maxes - The list of axis/motors that are moved (eg. "ABCD")
 * @param [in] paxes - The list of axis/motors that are prepared for a move (eg. "ABCD")
 * @return motor driver status code.
 */
asynStatus GalilController::beginGroupMotion(const char *maxes, const char *paxes)
{
  const char *functionName = "beginGroupMotion";
  string mesg;				//Controller mesg
  char allaxes[MAX_GALIL_AXES];		//Move axes, and prepareAxes list concatenated

  //Concatenate list of move axes, and prepare to move axes
  sprintf(allaxes,"%s%s", maxes, paxes);

  if (strcmp(allaxes, "") != 0) {
     //Execute motor auto on and brake off function
     executeAutoOnBrakeOff(allaxes);

     //Execute motor record prem
     executePrem(allaxes);

     if (strcmp(maxes, "") != 0) {
        //For move axes only, begin the move
        sprintf(cmd_, "BG %s", maxes);
        if (sync_writeReadController() == asynSuccess) {
           //Wait till moving true on all axes delivered to mr
           //or timeout occurs
           axesEventMonitor(maxes);
        }
        else {
           //Controller gave error at begin
           mesg = string(functionName) + ": Begin failure " + string(maxes);
           //Set controller error mesg monitor
           setCtrlError(mesg);
           return asynError;
        }
     }
  }

  //Success
  return asynSuccess;
}

/** Returns true if all motors in the provided list send move status true
  * to the motor record within timeout period else return false
  * called from beginGroupMotion, beginLinearGroupMotion, and beginPVTProfileMotion
  * \param[in] Motor list
  * \param[in] Requested event - 0 = beginEvent_, 1 = stopEvent_
  */
bool GalilController::axesEventMonitor(const char *axes, unsigned requestedEvent)
{
  int i;		//Looping
  bool result = true;	//Result
  GalilAxis *pAxis;	//GalilAxis instance

  if (!strcmp(axes, ""))
     return true;//No work

  //Configure, then start axis event monitors
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve the axis
     pAxis = getAxis(axes[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //Set the event to be monitored
     pAxis->requestedEvent_ = (requestedEvent == 0) ? pAxis->beginEvent_ : pAxis->stopEvent_;
     //Tell poller we want a signal when event happens
     pAxis->requestedEventSent_ = false;
     //Start event monitor for this axis
     epicsEventSignal(pAxis->eventMonitorStart_);
  }

  //Wait for event monitors to complete
  //We can receive events in a serial way without effecting the time taken
  //Thanks to axis eventMonitor thread

  //Poller sends the signal we wait on
  //Release lock so sync poller can get lock
  unlock();
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve the axis
     pAxis = getAxis(axes[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //Wait for event monitor done signal
     epicsEventWait(pAxis->eventMonitorDone_);
     //Check for timeout, or error on this axis
     if (pAxis->eventResult_ != epicsEventWaitOK) {
        //Set result flag, and continue to receive all eventMonitorDone_ events
        result = false;
     }
  }

  //Retake lock
  lock();

  //Return result
  return result;
}

/** Returns true if all motors in the provided list send move status true
  * to the motor record within timeout period else return false
  * called from setDeferredMoves
  * \param[in] Motor list
  */
bool GalilController::csaxesBeginMonitor(const char *axes)
{
  int i;		//Looping
  bool result = true;	//Result
  GalilCSAxis *pCSAxis;	//GalilCSAxis instance

  if (!strcmp(axes, ""))
     return true;//No work

  //Configure, then start axis event monitors
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve the axis
     pCSAxis = getCSAxis(axes[i] - AASCII);
     //Process or skip
     if (!pCSAxis) continue;
     //Set the event to be monitored
     pCSAxis->requestedEvent_ = pCSAxis->beginEvent_;
     //Tell poller we want a signal when event happens
     pCSAxis->requestedEventSent_ = false;
     //Start event monitor for this axis
     epicsEventSignal(pCSAxis->eventMonitorStart_);
  }

  //Wait for event monitors to complete
  //We can receive events in a serial way without effecting the time taken
  //Thanks to axis eventMonitor thread

  //Poller sends the signal we wait on
  //Release lock so sync poller can get lock
  unlock();
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve the axis
     pCSAxis = getCSAxis(axes[i] - AASCII);
     //Process or skip
     if (!pCSAxis) continue;
     //Wait for begin monitor done signal
     epicsEventWait(pCSAxis->eventMonitorDone_);
     //Check for timeout, or error on this axis
     if (pCSAxis->eventResult_ != epicsEventWaitOK) {
        //Set result flag, and continue to receive all eventMonitorDone_ events
        result = false;
     }
  }

  //Retake lock
  lock();

  //Return result
  return result;
}

//Clear deferred move flag on axes group
void GalilController::clearDeferredMoves(const char *axes) {
  GalilAxis *pAxis;			//GalilAxis instance
  unsigned i;				//Looping

  //Loop through the axes list
  //Turn off deferredMove_ for these axis
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve axis specified in axes list
     pAxis = getAxis(axes[i] - AASCII);
     //Process or skip
     if (!pAxis) continue;
     //Set flag
     pAxis->deferredMove_ = false;
  }
}

/**
 * Perform a deferred move (a coordinated group move)
 * Motor start times are synchronized
 * @param [in] axes - The list of axis/motors in the move (eg. "ABCD")
 * @return motor driver status code.
 */
asynStatus GalilController::beginSyncStartOnlyMove(const char *axes)
{
  GalilAxis *pAxis;		//GalilAxis instance
  string cmd = "";		//Command string
  bool first = true;		//First loop doesn't require initial ';'
  unsigned i;			//looping
  asynStatus status;		//Return status

  //Loop thru axes, set acceleration, setpoints
  for (i = 0; axes[i] != '\0'; i++) {
     //Retrieve the axis
     pAxis = getAxis(axes[i] - AASCII);
     //Skip loop if !pAxis 
     if (!pAxis) continue;
     //Check axis for Sync motor start only deferred move
     if (pAxis->deferredMove_ && !pAxis->deferredMode_) {
        //Axis has a sync start only deferred move
        //Set the acceleration and velocity for this axis
        pAxis->setAccelVelocity(pAxis->deferredAcceleration_, pAxis->deferredVelocity_);
        //Add semi colon as required
        if (first)
           first = false;
        else
           cmd += ";";
        //Assemble command to set position
        if (pAxis->deferredRelative_)
           cmd += "PR";
        else
           cmd += "PA";
        cmd += string(1, pAxis->axisName_) + "=" + tsp(pAxis->deferredPosition_, 0);
        //Check if column getting too wide, write data if needed
        if (cmd.size() > 63) {
           strcpy(cmd_, cmd.c_str());
           cmd.clear();
           sync_writeReadController();
        }
     }
  } //For (axis)

  //Flush remaining position setpoints to controller
  if (cmd.size() > 0) {
     strcpy(cmd_, cmd.c_str());
     cmd.clear();
     sync_writeReadController();
  }

  //Start the group of motors
  status = beginGroupMotion(axes);

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
   string axes = "";			//Constructed list of axis in the deferred move
   string cmd = "";			//Command string
   bool groupokay = true;		//If axis has problem, then reject group move

   //Loop through the axis looking for deferredMoves
   //Check axis enable, if problem set flag to later reject entire group move
   //Construct axis list
   for (axis = 0; axis < MAX_GALIL_AXES; axis++) {
      //Retrieve the axis
      pAxis = getAxis(axis);
      //Skip loop if !pAxis 
      if (!pAxis) continue;
      //Check axis for Sync motor start only deferred move
      if (pAxis->deferredMove_ && !pAxis->deferredMode_) {
         //Deferred move found
         //Check axis is good to go
         if (pAxis->beginCheck(functionName, pAxis->axisName_, pAxis->deferredVelocity_, false)) {
            //Axis has problem, reject group move
            groupokay = false;
         }
         //Store axis in list
         axes += pAxis->axisName_;
      }
   } //For (axis)

   if (groupokay && !axes.empty()) {
      //All axis in group okay, axes list not empty
      //Start the sync start only move
      beginSyncStartOnlyMove(axes.c_str());
   }

   //Turn off deferredMove_ for these axes
   clearDeferredMoves(axes.c_str());

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
  unsigned i;			//Axis looping
  string csaxes = "";		//CSAxis list

  //If we are not ending deferred moves then return
  if (deferMoves || !movesDeferred_)
     {
     movesDeferred_ = true;
     return asynSuccess;
     }

  //We are ending deferred moves.  So process them
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "Processing deferred moves on Galil: %s\n", this->portName);

  //Ending deferred moves
  //Clear movesDeferred_ flag
  movesDeferred_ = false;

  //Execute deferred moves
  //Sync start and stop moves
  prepSyncStartStopMoves();
  //Sync start only moves
  prepSyncStartOnlyMoves();

  //Construct CSAxis list with deferred move
  for (i = MAX_GALIL_AXES; i < MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++) {
     //Retrieve the CSAxis
     pCSAxis = getCSAxis(i);
     //Process or skip
     if (!pCSAxis) continue;
     if (pCSAxis->deferredMove_) {
        //CSAxis has a deferred move
        //Construct list of axis with deferred move
        csaxes += pCSAxis->axisName_;
     }
  }

  //Wait till CSAxis move status true delivered to MR
  //Real motion only, deferredMove ignored
  csaxesBeginMonitor(csaxes.c_str());

  //Clear CSAxis deferredMove_ flag
  for (string::iterator it = csaxes.begin(); it != csaxes.end(); it++) {
     //Retrieve axis instance
     pCSAxis = getCSAxis(*it - AASCII);
     if (pCSAxis->deferredMove_) {
        //Clear deferredMoves flag for this csaxis
        pCSAxis->deferredMove_ = false;
     }
  }

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
   int function = pasynUser->reason;		//function requested
   asynStatus status;				//Used to work out communication_error_ status.  asynSuccess always returned
   GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
   int ecatcapable;				//EtherCat capable status
   unsigned i;					//Looping
   double currentLoopGain;                      //Current loop gain
   int addr;
   status = getAddress(pasynUser, &addr);

   //Just return if shutting down
   if (shuttingDown_)
      return asynSuccess;

   //We dont retrieve values for records at iocInit.  
   //For output records autosave, or db defaults are pushed to hardware instead
   if (!dbInitialized) return asynError;
    
   if (function == GalilLimitType_) {
      //Read limit type from controller
      strcpy(cmd_, "MG _CN0");
      status = get_integer(GalilLimitType_, value);
      if (!status)
         *value = (*value > 0) ? 1 : 0;
   }
   else if (function == GalilMainEncoder_ || function == GalilAuxEncoder_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      unsigned setting;
      int main, aux;
	
      sprintf(cmd_ , "CE%c=?", pAxis->axisName_);
      if ((status = sync_writeReadController()) == asynSuccess) {
         setting = (unsigned)atoi(resp_);
         //Separate setting into main and aux
         main = setting & 3;
         aux = setting & 12;
         *value = (function == GalilMainEncoder_) ? main : aux;
      }
      else {
         //Comms error, return last ParamList value set using setIntegerParam
         if (function == GalilMainEncoder_) {
            getIntegerParam(pAxis->axisNo_, GalilMainEncoder_, &main);
            *value = main;
         }
         else {
            getIntegerParam(pAxis->axisNo_, GalilAuxEncoder_, &aux);
            *value = aux;
         }
      }
   }
   else if (function == GalilMotorType_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      float motorType;
      sprintf(cmd_, "MG _MT%c", pAxis->axisName_);
      if ((status = sync_writeReadController()) == asynSuccess) {
         motorType = (float)atof(resp_);
         //Upscale by factor 10 to create integer representing motor type 
         *value = (int)(motorType * 10.0);
         //Translate motor type into 0-12 value for mbbi record
         switch (*value) {
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
            case 100:  *value = 8;
                       break;
            case 110:  *value = 9;
                       break;
            case -110: *value = 10;
                       break;
            case 40:   *value = 11;
                       break;
            case -40:  *value = 12;
                       break;
            default:   break;
         }//Switch
      }
      else    //Comms error, return last ParamList value set using setIntegerParam
         getIntegerParam(pAxis->axisNo_, function, value);
   } //GalilMotorType_
   else if (function == GalilBrushType_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _BR%c", pAxis->axisName_);
      status = get_integer(GalilBrushType_, value);
   }
   else if (function == GalilAmpModel_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      *value = (addr <= 3)? ampModel_[0] : ampModel_[1];
   }
   else if (function >= GalilSSIInput_ && function <= GalilSSIData_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      int ssicapable;	//Local copy of GalilSSICapable_
      //Retrieve GalilSSICapable_ param
      getIntegerParam(GalilSSICapable_, &ssicapable);
      if (ssicapable)
         status = pAxis->get_ssi(function, value);
      else
         status = asynSuccess;
   }
   else if (function >= GalilBISSInput_ && function <= GalilBISSLevel_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      int bisscapable = 0;
      getIntegerParam(GalilBISSCapable_, &bisscapable);
      if (bisscapable)
         if (function == GalilBISSLevel_) {
            sprintf(cmd_, "MG _SY%c", pAxis->axisName_);
            status = get_integer(GalilBISSLevel_, value);
         } 
         else {
            status = pAxis->get_biss(function, value);
         }
	 else
            status = asynSuccess;
   }
   else if (function == GalilCoordSys_) {
      //Read active coordinate system
      sprintf(cmd_, "MG _CA");
      status = get_integer(GalilCoordSys_, value);
      if (!status)
         *value = (*value > 0) ? 1 : 0;
   }
   else if (function == GalilOffOnError_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _OE%c", pAxis->axisName_);
      status = get_integer(GalilOffOnError_, value);
   }
   else if (function == GalilMicrostep_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _YA%c", pAxis->axisName_);
      status = get_integer(GalilMicrostep_, value);
   }
   else if (function == GalilAmpGain_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _AG%c", pAxis->axisName_);
      status = get_integer(GalilAmpGain_, value);
   }
   else if (function == GalilAmpCurrentLoopGain_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _AU%c", pAxis->axisName_);
      if ((status = sync_writeReadController()) == asynSuccess) {
         currentLoopGain = (double)atof(resp_);
         // Translate currentLoopGain to mbbi enum values
         if (currentLoopGain == 0)
            *value = 0;
         else if (currentLoopGain == 0.5)
            *value = 1;
         else if (currentLoopGain == 1)
            *value = 2;
         else if (currentLoopGain == 1.5)
            *value = 3;
         else if (currentLoopGain == 2)
            *value = 4;
         else if (currentLoopGain == 3)
            *value = 5;
         else if (currentLoopGain == 4)
            *value = 6;
         else if (currentLoopGain == 9)
            *value = 7;
         else if (currentLoopGain == 10)
            *value = 8;
         else if (currentLoopGain == 11)
            *value = 9;
         else if (currentLoopGain == 12)
            *value = 10;
      }
   }
   else if (function == GalilAmpLowCurrent_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      sprintf(cmd_, "MG _LC%c", pAxis->axisName_);
      status = get_integer(GalilAmpLowCurrent_, value);
   }
   else if (function == GalilLimitDisable_) {
      //If provided addr does not return an GalilAxis instance, then return asynError
      if (!pAxis) return asynError;
      if (model_[3] != '1' && model_[3] != '2') {
         sprintf(cmd_, "MG _LD%c", pAxis->axisName_);
         status = get_integer(GalilLimitDisable_, value);
      }
   }
   else if (function == GalilCtrlEtherCatFault_) {
      //Retrieve required params
      getIntegerParam(GalilEtherCatCapable_, &ecatcapable);
      if (ecatcapable) {
         //Controller is EtherCat capable
         //Determine EtherCat fault code
         strcpy(cmd_, "MG _EU1");
         status = get_integer(GalilCtrlEtherCatFault_, value);
         //Set axis EtherCat fault status PV's
         for (i = 0; i < MAX_GALIL_AXES; i++) {
            if (*value & (1 << i))
               setIntegerParam(i, GalilEtherCatFault_, 1);
            else
               setIntegerParam(i, GalilEtherCatFault_, 0);
         }
      }
   }
   else if (function == GalilEtherCatNetwork_) {
      //Retrieve required params
      getIntegerParam(GalilEtherCatCapable_, &ecatcapable);
      if (ecatcapable) {
         //Controller is EtherCat capable
         //Determine if EtherCat network up
         sprintf(cmd_, "MG _EU0");
         status = get_integer(GalilEtherCatNetwork_, value);
      }
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

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

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
     //For when input records are set to periodic scan not I/O Intr
     sprintf(cmd_, "MG @AN%d", addr);
     get_double(GalilAnalogIn_, value, addr);
     }
  else  //Command not found
     asynPortDriver::readFloat64(pasynUser, value);

  //Always return success. Dont need more error mesgs
  return asynSuccess;	
}

/** Called when asyn clients call pasynEnum->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] strings Array of string pointers.
  * \param[in] values Array of values
  * \param[in] severities Array of severities
  * \param[in] nElements Size of value array
  * \param[out] nIn Number of elements actually returned */
asynStatus GalilController::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[], size_t nElements, size_t *nIn)
{
  int function = pasynUser->reason;
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  int i;
  int boardNum;
  const enumStruct_t *pEnum;
  int numEnums;
  //static const char *functionName = "readEnum";

  //If provided pasynUser does not return an GalilAxis instance, then return asynError
  if (!pAxis) goto unsupported;

  if ((pAxis->axisName_ >= 'A') && (pAxis->axisName_ <= 'D')) {
    boardNum = 0;
  }
  else if ((pAxis->axisName_ >= 'E') && (pAxis->axisName_ <= 'H')) {
    boardNum = 1;
  }
  else {
    goto unsupported;
  }

  if (function == GalilAmpGain_) {   
    switch (ampModel_[boardNum]) {
      case 0:
        // No amplifier
        goto unsupported;
      case 43020:
      case 43040: 
        pEnum    = ampGain_43040;
        numEnums = sizeof(ampGain_43040)/sizeof(enumStruct_t);
        break;
      case 43140:
        pEnum    = ampGain_43140;
        numEnums = sizeof(ampGain_43140)/sizeof(enumStruct_t);
        break;
      case 43547:
        // This cannot be handled here because the allowed gains depend on the selected motor type
        // We use doCallBacksEnum when the motor type changes
        goto unsupported; 
      case 44040:
        pEnum    = ampGain_44040;
        numEnums = sizeof(ampGain_44040)/sizeof(enumStruct_t);
        break;
      case 44140:
        pEnum    = ampGain_44140;
        numEnums = sizeof(ampGain_44140)/sizeof(enumStruct_t);
        break;
      default:
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "GalilController::readEnum unknown amplifier gain model=%d, address=%s, axis=%c\n", 
                  ampModel_[boardNum], address_.c_str(), pAxis->axisName_);
        goto unsupported;
    }
  }
  else if (function == GalilMicrostep_) {   
    switch (ampModel_[boardNum]) {
      case 0:
        // No amplifier
        goto unsupported;
      case 43020:
      case 43040:
      case 43140:
        // These are servo only, no microstep feature
        goto unsupported;
      case 43547:
        pEnum    = microstep_43547;
        numEnums = sizeof(microstep_43547)/sizeof(enumStruct_t);
        break;
      case 44040:
        pEnum    = microstep_44040;
        numEnums = sizeof(microstep_44040)/sizeof(enumStruct_t);
        break;
      case 44140:
        pEnum    = microstep_44140;
        numEnums = sizeof(microstep_44140)/sizeof(enumStruct_t);
        break;
      default:
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "GalilController::readEnum unknown amplifier microstep model=%d, address=%s, axis=%c\n", 
                  ampModel_[boardNum], address_.c_str(), pAxis->axisName_);
        goto unsupported;
    }
  }
  else {
    goto unsupported;
  }
  for (i=0; ((i<numEnums) && (i<(int)nElements)); i++) {
    if (strings[i]) free(strings[i]);
    strings[i] = epicsStrDup(pEnum[i].enumString);
    values[i] = pEnum[i].enumValue;
    severities[i] = 0;
  }
  *nIn = i;
  return asynSuccess;

unsupported:
  *nIn = 0;
  return asynError;
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

   if (function == GalilBinaryOut_) {
      //Ensure record mask is within range
      maxmask = (rio_) ? 0x80 : 0x8000;

      if (mask > maxmask && model_.find("Unknown") == string::npos) {
         printf("%s model %s mask too high @ > %x addr %d mask %x\n", functionName, model_.c_str(), maxmask, addr, mask);
         return asynSuccess;
      }
		
      //Determine bit i from mask
      for (i = 0; i < 16; i++) {
         if ((epicsUInt32)(1 << i) == mask) {
            //Bit numbering is different on DMC compared to RIO controllers
            if (!rio_) {
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
            //We found the correct bit, return
            return asynSuccess;
         }
      } //For
   } //Function GalilBinaryOut_

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
  int addr = 0;				        //Address requested
  int status;					//Used to work out communication_error_ status.  asynSuccess always returned
  GalilAxis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  GalilCSAxis *pCSAxis = getCSAxis(pasynUser);	//Retrieve the axis instance
  int hometype, limittype;			//The home, and limit switch type
  int homeedge;					//Home edge to locate
  int homesetting;				//Controller home switch _CN1 setting
  int mainencoder, auxencoder, encoder_setting; //Main, aux encoder setting
  char coordinate_system;			//Coordinate system S or T
  char axes[MAX_GALIL_AXES];			//Coordinate system axis list
  string mesg = "";				//Controller mesg
  double eres, mres;				//mr eres and mres
  double hvel, accl;				//mr hvel, accl
  double acceleration;				//Axis acceleration Units=steps/sec/sec
  int spmg;					//mr stop pause move go (spmg)
  float oldmotor;				//Motor type before changing it.  Use Galil numbering
  unsigned i;					//Looping
  float oldmtr_abs, newmtr_abs;			//Track motor changes
  int uploading;				//Array uploading status
  int wlp;					//Wrong limit protection setting
  int limitDisable;				//Limit disable setting
  bool ctrlType;				//Controller type convenience variable

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return((asynStatus)status);

  //Check axis instance
  if (addr < MAX_GALIL_AXES && !rio_) {
     if (!pAxis) return asynError;
  }
  else if (!rio_) {
     if (!pCSAxis) return asynError;
  }

  //Protect against calling upload array whilst status is uploading
  getIntegerParam(GalilUserArrayUpload_, &uploading);
  if (uploading && function == GalilUserArrayUpload_)//User requesting upload, but its still running
     return asynSuccess;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(addr, function, value);
  
  if (function == GalilHomeType_ || function == GalilHomeEdge_ || function == GalilLimitType_) {
     //Retrieve required parameters
     status = getIntegerParam(GalilHomeType_, &hometype);
     status |= getIntegerParam(GalilHomeEdge_, &homeedge);
     status |= getIntegerParam(GalilLimitType_, &limittype);
     if (!status) {
        //Convert limit type to controller syntax
        limittype = (limittype > 0) ? 1 : -1;
        //Determine controller home setting for _CN1
        homesetting = ((!hometype && !homeedge)||(hometype && homeedge)) ? 1 : -1;
        //Determine home switch active, inactive value for _HM operand
        hswact_ = ((!hometype && !homeedge)||(hometype && !homeedge)) ? 1 : 0;
        hswiact_ = ((!hometype && homeedge)||(hometype && homeedge)) ? 1 : 0;
        //Set limit and home switch configuration
        sprintf(cmd_, "CN %d,%d,-1,0,1", limittype, homesetting);
        //Write setting to controller
        status = sync_writeReadController();
        //Update home edge variable on controller
        sprintf(cmd_, "hswedg=%d", homeedge);
        //Write setting to controller
        status = sync_writeReadController();
     }
  }
  else if (function == GalilAuxEncoder_ || function == GalilMainEncoder_) {
     //Retrieve main and aux encoder setting
     //Must ensure getIntegerParam successful, as some parameters may not be set yet due to record default mechanism
     if ((getIntegerParam(pAxis->axisNo_, GalilMainEncoder_, &mainencoder) == asynSuccess) && (getIntegerParam(pAxis->axisNo_, GalilAuxEncoder_, &auxencoder) == asynSuccess)) {
        //Assemble cmd string 
        encoder_setting = mainencoder + auxencoder;
        sprintf(cmd_, "CE%c=%d", pAxis->axisName_, encoder_setting);
        //Write setting to controller
        status = sync_writeReadController();
     }
  }
  else if (function == GalilMotorType_) {
     float newmtr;

     //Query motor type before changing setting
     sprintf(cmd_, "MT%c=?", pAxis->axisName_);
     sync_writeReadController();
     oldmotor = (float)atof(resp_);

     //Assemble command to change motor type
     switch (value) {
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
        case 8: newmtr = 10;
                break;
        case 9: newmtr = 11;
                break;
        case 10: newmtr = -11;
                break;
        case 11: newmtr = 4;
                break;
        case 12: newmtr = -4;
                break;
        default: newmtr = 1.0;
                break;
     } //Switch

     //Change motor type
     sprintf(cmd_, "MT%c=%1.1f", pAxis->axisName_, newmtr);
     //Write setting to controller
     status = sync_writeReadController();

     //Determine motor type
     pAxis->motorIsServo_ = (value < 2 || (value >= 6 && value <= 12)) ? true : false;

     //IF motor was servo, and now stepper
     //Galil hardware MAY push main encoder to aux encoder (stepper count reg)
     //We re-do this, but apply encoder/motor scaling
     if ((fabs(oldmotor) <= 1.5 || fabs(oldmotor) == 4.0) && value >= 2 && value <= 5) {
        getDoubleParam(pAxis->axisNo_, GalilEncoderResolution_, &eres);
        getDoubleParam(pAxis->axisNo_, motorResolution_, &mres);
        if (mres != 0.000000) {
           //Calculate step count from existing encoder_position, construct mesg to controller_
           sprintf(cmd_, "DP%c=%.0lf", pAxis->axisName_, pAxis->encoder_position_ * (eres/mres));
           //Write setting to controller
           status = sync_writeReadController();
        }
     }

     //IF motor was stepper, and now servo
     //Set reference position equal to main encoder, which sets initial error to 0
     if ((fabs(oldmotor) >= 2.0 && fabs(oldmotor) <= 2.5) &&
        ((value < 2) || (value >= 6 && value <= 7) || (value >= 11 && value <= 12))) {
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
		((oldmotor == -1.0 && newmtr == 1.5) || (oldmtr_abs == 1.5 && newmtr_abs == -1.0)) ||
		((oldmotor == -4.0 && newmtr == 4.0) || (oldmtr_abs == 4.0 && newmtr_abs == -4.0)) ||
		((oldmotor == -11 && newmtr == 11) || (oldmotor == 11 && newmtr == -11))) {
        //New motor is same as old but opposite direction
        if (pAxis->limitsDirState_ != unknown) {
           //Toggle motor/limit direction consistency since it was known
           pAxis->limitsDirState_ = (pAxis->limitsDirState_ == not_consistent) ? consistent : not_consistent;
           //Update motor/limit direction consistency in paramList
           setIntegerParam(pAxis->axisNo_, GalilLimitConsistent_, pAxis->limitsDirState_);
        }
     }
     else if (oldmotor != newmtr)
        pAxis->limitsDirState_ = unknown;
        
     // On some amplifiers (e.g. 43547) changing the motor type changes the amplifier gain enums.
     // Need to do callbacks with the new enum choices.
     ampGainCallback(addr, value);
  }
  else if (function == GalilBrushType_) {
     sprintf(cmd_, "BR%c=%d", pAxis->axisName_, value);
     //Write setting to controller
     status = sync_writeReadController();
  }
  else if (function == GalilUseEncoder_) {
     //This is one of the last items pushed into driver at startup so flag
     //Axis now ready for move commands
     if (addr < MAX_GALIL_AXES) {
        pAxis->axisReady_ = true;//Real motor
     }
     else
        pCSAxis->axisReady_ = true;//CS motor
  }
  else if (function >= GalilSSIInput_ && function <= GalilSSIData_) {
     int ssicapable;	//Local copy of GalilSSICapable_
     //Retrieve GalilSSICapable_ param
     getIntegerParam(GalilSSICapable_, &ssicapable);
	
     //Only if controller is SSI capable
     if (ssicapable)
        status = pAxis->set_ssi();
  }
  else if (function >= GalilBISSInput_ && function <= GalilBISSLevel_) {
     int bisscapable = 0;
     getIntegerParam(GalilBISSCapable_, &bisscapable);
     if (bisscapable) {
        if (function == GalilBISSLevel_) {
           sprintf(cmd_, "SY%c=%d", pAxis->axisName_, value);
           //Write setting to controller
           status = sync_writeReadController();
        }
        else {
           status = pAxis->set_biss();
        }
     }
  }
  else if (function == GalilSSIInvert_) {
     pAxis->invert_ssi_ = (value == 0) ? false : true;
  }
  else if (function == GalilOffOnError_) {
     sprintf(cmd_, "OE%c=%d", pAxis->axisName_, value);
     //Write setting to controller
     status = sync_writeReadController();
  }
  else if (function == GalilCoordSys_) {
     coordinate_system = (value == 0) ? 'S' : 'T';
     sprintf(cmd_, "CA %c", coordinate_system);
     //Write setting to controller
     status = sync_writeReadController();
  }
  else if (function == GalilCoordSysMotorsStop_) {
     //Retrieve coordSys axes list
     getStringParam(addr, GalilCoordSysMotors_, MAX_GALIL_AXES, axes);
     //Stop axis motor records
     //This is done to stop motor record backlash, and retries
     for (i = 0; axes[i] != '\0'; i++) {
        //Retrieve axis instance
        pAxis = getAxis(axes[i] - AASCII);
        //Skip or process
        if (!pAxis) continue;
        //Set flag indicating axis motor record will be stopped by driver
        pAxis->stopInternal_ = true;
        //Stop axis motor record
        pAxis->stopMotorRecord();
     }
     //Stop coordsys
     sprintf(cmd_, "ST %c", (addr == 0) ? 'S' : 'T');
     //Write setting to controller
     status = sync_writeReadController();
  }
  else if (function == GalilOutputCompareAxis_) {
     status = setOutputCompare(addr);
  }
  else if (function == GalilUserArrayUpload_) {
     epicsEventSignal(arrayUploadEvent_);
  }
  else if (function == GalilMicrostep_) {
     //Set microsteps/step
     sprintf(cmd_, "YA%c=%d", pAxis->axisName_, value);
     sync_writeReadController();
  }
  else if (function == GalilAmpGain_) {
     int motorOff = 0;
     //Retrieve motor off status
     sprintf(cmd_, "MG _MO%c", pAxis->axisName_);
     status = sync_writeReadController();
     if (!status)
        motorOff = atoi(resp_);
     if (!motorOff) {
        //Motor must be off to change gain
        sprintf(cmd_, "MO%c", pAxis->axisName_);
        sync_writeReadController();
        }
     //Motor is now off, we can change gain
     sprintf(cmd_, "AG%c=%d", pAxis->axisName_, value);
     sync_writeReadController();
     if (!motorOff) {
        //Motor was on at start, so turn it back on
        sprintf(cmd_, "SH%c", pAxis->axisName_);
        sync_writeReadController();
     }
  }
  else if (function == GalilAmpCurrentLoopGain_) {
     float gainSetting = 0;
     //Find the correct gain setting
     if (value == 0)
        gainSetting = 0;
     else if (value == 1)
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
     else if (value == 7)
        gainSetting = 9;
     else if (value == 8)
        gainSetting = 10;
     else if (value == 9)
        gainSetting = 11;
     else if (value == 10)
        gainSetting = 12;
     //Set current loop gain
     sprintf(cmd_, "AU%c=%.1f", pAxis->axisName_, gainSetting);
     sync_writeReadController();
  }
  else if (function == GalilAmpLowCurrent_) {
     //Set low current mode
     sprintf(cmd_, "LC%c=%d", pAxis->axisName_, value);
     sync_writeReadController();
  }
  else if (function == GalilLimitDisable_) {
     //Clear controller message
     if (pAxis->axisReady_)
        setCtrlError("");
     //Construct controller type convenience variable
     ctrlType = (bool)(model_[3] != '2' && model_[3] != '1') ? true : false;
     if (ctrlType) {
        //Enable/Disable the limits
        sprintf(cmd_, "LD%c=%d", pAxis->axisName_, value);
        sync_writeReadController();
        getIntegerParam(pAxis->axisNo_, GalilWrongLimitProtection_, &wlp);
        //Check if limitDisable conflicts with WLP
        if (wlp && value > 0) {
           mesg = address_ + " Axis " + string(1, pAxis->axisName_) + " wrong limit protect disabled at end(s) with disabled limits";
           setCtrlError(mesg);
        }
     }
     else if (value != 0) {
        //Controller doesn't support limit disable
        mesg = address_ + " Axis " + pAxis->axisName_ + " does not support limit disable feature";
        setCtrlError(mesg);
        setIntegerParam(pAxis->axisNo_, GalilLimitDisable_, 0);
     }
  }
  else if (function == GalilWrongLimitProtection_) {
     //Clear controller message
     if (pAxis->axisReady_)
        setCtrlError("");
     getIntegerParam(pAxis->axisNo_, GalilLimitDisable_, &limitDisable);
     //Check if WLP conflicts with limitDisable
     if (limitDisable && value) {
        mesg = address_ + " Axis " + string(1, pAxis->axisName_) + " wrong limit protect disabled at end(s) with disabled limits";
        setCtrlError(mesg);
     }
  }
  else if (function == GalilEtherCatNetwork_) {
        enableEtherCatNetwork(value);
  }
  else if (function == GalilEtherCatFaultReset_) {
        pAxis->clearEtherCatFault();
  }
  else if (function == GalilEtherCatAddress_) {
     int ecatcapable;
     getIntegerParam(GalilEtherCatCapable_, &ecatcapable);
     if (ecatcapable) {
        //Controller is EtherCat capable, set axis drive address
        sprintf(cmd_, "EX%c=%d", pAxis->axisName_, value); 
        sync_writeReadController();
     }
  }
  else if (function == GalilHomr_ || function == GalilHomf_) {
     //Retrieve required parameters
     status = getIntegerParam(pAxis->axisNo_, GalilStopPauseMoveGo_, &spmg);
     status |= getDoubleParam(pAxis->axisNo_, GalilMotorHvel_, &hvel);
     status |= getDoubleParam(pAxis->axisNo_, motorResolution_, &mres);
     if (!status && spmg == spmgGo) {
        //Motor record stop pause move go is set GO
        //Convert Hvel from EGU/sec to steps/sec to provide maxVelocity to home method
        hvel = hvel / mres;
        //Check motor record has set acceleration
        if (getDoubleParam(pAxis->axisNo_, motorAccel_, &acceleration) == asynParamUndefined) {
           if (getDoubleParam(pAxis->axisNo_, GalilMotorAccl_, &accl) == asynSuccess) {
              //asynMotorController has not set motor acceleration parameter motorAccel_
              //Calculate and set acceleration parameter motorAccel_
              //asynMotorController uses parameter motorAccel_ to set acceleration 
              //used in GalilAxis method calls
              acceleration = hvel / accl;
              setDoubleParam(pAxis->axisNo_, motorAccel_, acceleration);
           }
           else //Acceleration when it can't be calculated due to paramList error
              acceleration = 4096;
        }
        //Call axis home
        pAxis->home(0, hvel, acceleration, (function == GalilHomr_) ? 0 : 1);
     }
  }
  else if (function == GalilClearAmpFaults_) {
     //Controller must be 40xx or 41xx series
     if (model_[3] == '4' && (model_[4] == '0' || model_[4] == '1')) {
        //Clear all latched amplifier errors
        strcpy(cmd_, "AZ1");
        status = sync_writeReadController();
        if (asynSuccess != status) {
           //Command failed, get error message from controller
           strcpy(cmd_, "TC1");
           if (asynSuccess == (status = sync_writeReadController())) {
              //Set error message
              setCtrlError(resp_);
           }
        }
     }
  }
  else {
     /* Call base class method */
     status = asynMotorController::writeInt32(pasynUser, value);
  }

  //Always return success. Dont need more error mesgs
  return (asynStatus)asynSuccess;
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

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

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
        status = sync_writeReadController();
        }
     }
  else if (function == GalilErrorLimit_)
     {
     if (pAxis)
        {
        //Write new error limit to GalilController
        sprintf(cmd_, "ER%c=%lf",pAxis->axisName_, value);
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

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

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
  string mesg;					//Controller mesg
  GalilCSAxis *pCSAxis;				//Pointer to CSAxis instance
  int addr=0;					//Address requested
  char *endptr;					//String to double conversion

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

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
     //Set default response to empty string
     setStringParam(GalilUserOctet_, "");
     //Check user command for commands known to take a long time, increase timeout accordingly
     if (((value_s.find("BZ") != string::npos) && (value_s.find("=") != string::npos)) || 
          value_s.find("BP") != string::npos || value_s.find("BN") != string::npos || 
          value_s.find("BV") != string::npos) {
        //Increase timeout for user command
        timeout_ = 3;
     }

     //Ensure operator doesn't accidently kill all threads
     if (value_s.find("ST") == string::npos)
        {
        //Send the user command
        epicsSnprintf(cmd_, sizeof(cmd_), "%s", value_s.c_str());
        status = sync_writeReadController();

        //User command complete, set timeout back to default 1
        timeout_ = 1;
        if (status == asynSuccess)
           {
           //Set readback value(s) = response from controller
           //String monitor
           setStringParam(GalilUserOctet_, resp_);
           //ai monitor
           aivalue = strtod(resp_, &endptr);
           //Check for conversion error
           if (errno == 0 && *endptr == '\0') {
              //Conversion ok, pass value on
              setDoubleParam(0, GalilUserOctetVal_, aivalue);
           }
           //Determine if custom command had potential to alter controller time base
           if (value_s.find("TM ") != string::npos)
              {
              //Retrieve controller time base
              sprintf(cmd_, "MG _TM");
              if (sync_writeReadController() == asynSuccess)
                 timeMultiplier_ = DEFAULT_TIME / atof(resp_);
              }
           }
        else
           {
           //User command failed, get error message from controller
           strcpy(cmd_, "TC1");
           if ( (status = sync_writeReadController()) == asynSuccess )
              {
              //Set readback value = response from controller
              setStringParam(GalilUserOctet_, resp_);
              }
           }
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
           mesg = "Kinematics changed successfully";
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

/*Enable/Disable EtherCat network
  * \param[in] Enable, or disable ethercat network
*/
void GalilController::enableEtherCatNetwork(int value)
{
   string mesg;				//Controller mesg
   GalilAxis *pAxis;			//GalilAxis
   unsigned i;				//Looping
   int status;				//Asyn paramlist return success
   int ecatcapable;			//Controller ethercat capable
   int ecaterror;			//EtherCat error during attempt to bring up network
   bool ecatenabled = false;		//EtherCat network status
   int motorType;			//Motor type of individual axis

   //Is controller ethercat capable ?
   getIntegerParam(GalilEtherCatCapable_, &ecatcapable);
   if (ecatcapable)
      {
      //Controller is ethercat capable
      //Determine if EtherCat network up
      sprintf(cmd_, "MG _EU0");
      if (sync_writeReadController() == asynSuccess)
         ecatenabled = (bool)atoi(resp_);

      //Enable EtherCat
      if (value && !ecatenabled)
         {
         //Refresh controller knowledge of available drive addresses
         sprintf(cmd_, "EH");
         sync_writeReadController();
         //EtherCat timeout of 5000 ms
         sprintf(cmd_, "EU1<5000");
         if (sync_writeReadController() != asynSuccess)
            {
            //Something went wrong, determine error code
            sprintf(cmd_, "MG _TC");
            //Determine error string
            if (sync_writeReadController() != asynSuccess)
               {
               ecaterror = atoi(resp_);
               switch (ecaterror)
                  {
                  case ECAT_DUPLICATEID:mesg = "EtherCat: Error duplicate EtherCat address on network";
                                        break;
                  case ECAT_TIMEOUT:    mesg = "EtherCat: Error timeout";
                                        break;
                  case ECAT_DRIVENOTFOUND:
                                        mesg = "EtherCat: Error a drive address specified was not found";
                                        break;
                  case ECAT_NODRIVECONFIGURED:
                                        mesg = "EtherCat: Error EtherCat axis not configured";
                                        break;
                  default: break;
                  }
               }
            }
         else
            mesg = "EtherCat: Network enable successful";
         
         //Set the message
         setCtrlError(mesg);
         }

      //Disable EtherCat
      if (!value && ecatenabled)
         {
         //Turn off all ethercat motors
         for (i = 0; i < numAxesMax_; i++)
            {
            pAxis = NULL;
            status = getIntegerParam(i, GalilMotorType_, &motorType);
            if (!status)
               {
               if (motorType == 8 || motorType == 9 || motorType == 10)
                  {
                  //Motor is EtherCat type
                  pAxis = getAxis(i);
                  if (!pAxis) continue;
                  pAxis->setClosedLoop(false);
                  }
               }
            }
         //Disable ethercat network
         sprintf(cmd_, "EU0");
         sync_writeReadController();
         }
      }//EtherCat capable
}

/** Check axis physical limits given move request
  * \param[in] caller - Caller function name
  * \param[in] axes - List of axes
  * \param[out] axis - Axis that caused fault */
asynStatus GalilController::beginCheck(const char *caller, const char *axes, char *axis)
{
   GalilAxis *pAxis;
   unsigned i;

   //Check axes are ready to go
   for (i = 0; axes[i] != '\0'; i++) {
      //Retrieve the axis
      pAxis = getAxis(axes[i] - AASCII);
      if (!pAxis) continue;
      //Check axes are okay to go, supply arbitary velocity
      //Dont reset controller message
      if (pAxis->beginCheck(caller, pAxis->axisName_, 100, false)) {
         //Store axis that caused fault
         *axis = pAxis->axisName_;
         //Return if any error
         return asynError;
      }
   }

   //Return value
   return asynSuccess;
}

/** Check axis physical limits given move request
  * \param[in] caller - Caller function name
  * \param[in] axes - List of axes
  * \param[in] npos - List of axes new positions Units=steps
  * \param[out] axis - Axis that caused fault */
asynStatus GalilController::checkLimits(const char *caller, const char *axes, double npos[], char *axis) {
  GalilAxis *pAxis;	                //Galil axis instance
  int axisNo;                       //Axis number
  unsigned i;		                //Looping
  int moveMode;                     //Axis move mode

   //Check axes physical limits
   for (i = 0; axes[i] != '\0'; i++) {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve GalilProfileMoveMode_ from ParamList
      getIntegerParam(axisNo, GalilProfileMoveMode_, &moveMode);
      //Retrieve the axis
      pAxis = getAxis(axisNo);
      //Check limits only if profile move mode = absolute
      if (!pAxis || !moveMode) continue;
      if (pAxis->checkLimits(caller, axes[i], npos[i])) {
         //Store axis that caused fault
         *axis = pAxis->axisName_;
         //Return if any error
         return asynError;
      }
   }

  //Return status
  return asynSuccess;
}

/** Check axes soft limits given move request
  * \param[in] caller - Caller function name
  * \param[in] axes - List of axes
  * \param[in] npos - Reverse (real) axis new positions Units=steps
  * \param[out] axis - Axis that caused fault */
asynStatus GalilController::checkSoftLimits(const char *caller, const char *axes, double npos[], char *axis)
{
   GalilAxis *pAxis;		    //GalilAxis instance
   int axisNo;                  //Axis number
   unsigned i;      		    //Looping
   int moveMode;                //Axis move mode
   int status = asynSuccess;	//Return status

   //Loop thru reverse axis
   //check physical limits given move request
   for (i = 0; axes[i] != '\0'; i++) {
      //Determine axis number
      axisNo = axes[i] - AASCII;
      //Retrieve GalilProfileMoveMode_ from ParamList
      getIntegerParam(axisNo, GalilProfileMoveMode_, &moveMode);
      //Retrieve axis instance
      pAxis = getAxis(axisNo);
      //Check limits only if profile move mode = absolute
      if (!pAxis || !moveMode) continue;
      //Check axis physical limits
      if (pAxis->checkSoftLimits(caller, axes[i], npos[i])) {
         //Store axis that caused fault
         *axis = pAxis->axisName_;
         //Return if any error
         return asynError;
      }
   }

   //Return success
   return (asynStatus)status;
}

/** Check axes motor record settings
  * \param[in] caller - Caller function name
  * \param[in] axes - List of axes
  * \param[out] axis - Axis that caused fault */
asynStatus GalilController::checkMRSettings(const char *caller, const char *axes, char *axis)
{
   GalilAxis *pAxis;		//Real axis
   unsigned i;			//Looping

   //Check axes motor record status
   for (i = 0; axes[i] != '\0'; i++) {
      //Retrieve the axis
      pAxis = getAxis(axes[i] - AASCII);
      if (!pAxis) continue;
      //Check motor record settings
      if (pAxis->checkMRSettings(caller, pAxis->axisName_, false)) {
         //Store axis that caused fault
         *axis = pAxis->axisName_;
         return asynError;
      }
   }

   //Return value
   return asynSuccess;
}

/** Check all axes settings
  * \param[in] caller - Caller function name
  * \param[in] axes - List of axes
  * \param[in] npos - Axes new positions Units=Steps
  * \param[out] axis - Axis that caused fault */
asynStatus GalilController::checkAllSettings(const char *caller, const char *axes, double npos[], char *axis) {
   int status;              //Return status

   //Check profile axes MR settings
   status = checkMRSettings(caller, axes, axis);
   //Check axes physical limits given move
   status |= checkLimits(caller, axes, npos, axis);
   //Check axes soft limits given move
   status |= checkSoftLimits(caller, axes, npos, axis);
   //Check profile axes are ready to go
   status |= beginCheck(caller, axes, axis);

   //Return status
   return asynStatus(status);
}

/** Update installed amplifier information at controller connect
  * Called by GalilController::connected
  */
asynStatus GalilController::updateAmpInfo(void) {
  unsigned i; // Looping
  int status; // Return status
  char *pos; // Used to discover amplifier board
  const enumStruct_t *pEnum;
  int numEnums;

  //Determine what amplifier boards are installed
  strcpy(cmd_, "ID");
  ampModel_[0] = 0;
  ampModel_[1] = 0;
  //Get the ID string
  if ((status = sync_writeReadController()) == asynSuccess) {
    // Command successful
    // Determine amplifier boards installed
    pos = strstr(resp_, "AMP1,");
    if (pos) {
      ampModel_[0] = atoi(pos + strlen("AMP1,"));
    }
    pos = strstr(resp_, "AMP2,");
    if (pos) {
      ampModel_[1] = atoi(pos + strlen("AMP2,"));
    }
    // Is database initialized?
    if (dbInitialized) {
      // Database already initialized
      // Will have to update relevant mbbi, mbbo record enum states by callback instead
      // Cycle thru amplifier boards
      for (i = 0; i < 2; i++) {
        switch (ampModel_[i]) {
          case 43020:
          case 43040: 
            pEnum    = ampGain_43040;
            numEnums = sizeof(ampGain_43040)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilAmpGain_, pEnum, numEnums);
            break;
          case 43140:
            pEnum    = ampGain_43140;
            numEnums = sizeof(ampGain_43140)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilAmpGain_, pEnum, numEnums);
            break;
          case 43547:
            pEnum    = microstep_43547;
            numEnums = sizeof(microstep_43547)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilMicrostep_, pEnum, numEnums);
            // Gain is complicated for this model, different gains for stepper and servo modes
            break;
          case 44040:
            pEnum    = ampGain_44040;
            numEnums = sizeof(ampGain_44040)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilAmpGain_, pEnum, numEnums);
            pEnum    = microstep_44040;
            numEnums = sizeof(microstep_44040)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilMicrostep_, pEnum, numEnums);
            break;
          case 44140:
            pEnum    = ampGain_44140;
            numEnums = sizeof(ampGain_44140)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilAmpGain_, pEnum, numEnums);
            pEnum    = microstep_44140;
            numEnums = sizeof(microstep_44140)/sizeof(enumStruct_t);
            enumRowCallback(i, GalilMicrostep_, pEnum, numEnums);
            break;
        }
      }
    }
  }
  // Return status
  return (asynStatus)status;
}

/** Update installed amplifier information at controller connect
  * Called by GalilController::updateAmpInfo
  */
void GalilController::enumRowCallback(unsigned ampNum, int reason, const enumStruct_t *pEnum, size_t nElements) {
  unsigned i;
  unsigned addressStart;
  unsigned addressEnd;
  char *strings[MAX_ENUM_ROWS];
  int values[MAX_ENUM_ROWS];
  int severities[MAX_ENUM_ROWS];
  // Translate pEnum enumStruct_t into arrays of strings, values, severities for callback
  for (i = 0; ((i < nElements) && (i < MAX_ENUM_ROWS)); i++) {
    strings[i] = (char *)pEnum[i].enumString;
    values[i] = pEnum[i].enumValue;
    severities[i] = 0;
  }
  // From ampNum, determine relevant record addresses
  addressStart = (ampNum == 0) ? 0 : 4;
  addressEnd = (ampNum == 0) ? 4 : 8;
  // Send enum rows to records
  for (i = addressStart; i < addressEnd; i++) {
    doCallbacksEnum(strings, values, severities, nElements, reason, i);
  }
}

/** Update amplifier gains when motor type changes for some amplifiers (e.g. 43547)
  * Called by GalilController::writeInt32
  */
void GalilController::ampGainCallback(int axis, int motorType) {
  unsigned i;
  char *strings[MAX_ENUM_ROWS] = {0};
  int values[MAX_ENUM_ROWS];
  int severities[MAX_ENUM_ROWS];
  int ampModel;
  unsigned numEnums = 0;
  const enumStruct_t *pEnum;

  ampModel = (axis <= 3) ? ampModel_[0] : ampModel_[1];
  if (ampModel == 43547) {
    if ((motorType >= 1) && (motorType <= 5)) {
      pEnum    = ampStepperGain_43547;
      numEnums = sizeof(ampStepperGain_43547)/sizeof(enumStruct_t);
    } else {
      pEnum    = ampServoGain_43547;
      numEnums = sizeof(ampServoGain_43547)/sizeof(enumStruct_t);
    }
  }
  if (numEnums == 0) return;

  for (i = 0; ((i < numEnums) && (i < MAX_ENUM_ROWS)); i++) {
    if (strings[i]) free(strings[i]);
    strings[i] = epicsStrDup(pEnum[i].enumString);
    values[i] = pEnum[i].enumValue;
    severities[i] = 0;
  }
  doCallbacksEnum(strings, values, severities, numEnums, GalilAmpGain_, axis);
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
         memcpy(mesg, charstr, strlen(charstr));
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
               setCtrlError(rawbufOriginal);
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
   char src1[MAX_GALIL_STRING_SIZE];		//data source to retrieve
   char src2[MAX_GALIL_STRING_SIZE];		//Two data sources per loop supported
   int addr;					//addr or byte of IO
   int start, end;				//start, and end of analog numbering for this controller
   double paramDouble;				//For passing asynFloat64 to ParamList
   unsigned paramUDig;				//For passing UInt32Digital to ParamList
   unsigned in = 0, out = 0;			//For passing digital in/out for DMC30000 only
   int i;					//Looping
  
   //If data record query success in GalilController::acquireDataRecord
   if (recstatus_ == asynSuccess && connected_) {
      //extract relevant controller data from GalilController record, store in GalilController
      //If connected, then proceed

      //DMC30000 series only.
      if (model_[3] == '3') {
         //Specify data source 1, digital input bit
         strcpy(src1, "@IN[x]");
         //Specify data source 2, digital output bit
         strcpy(src2, "@OUT[x]");
         //First 8 input, and first 4 output bits only
         for (i = 1; i <= 8; i++) {
            //Specify input bit
            src1[4] = i + ZEROASCII;
            paramUDig = (unsigned)sourceValue(recdata_, src1);
            in += paramUDig << (i - 1);
            //Digital output bits
            if (i <= 4) {
               //Database records are arranged by word
               //ValueMask = 0xFFFF because a word is 16 bits
               //Specify output bit
               src2[5] = i + ZEROASCII;
               paramUDig = (unsigned)sourceValue(recdata_, src2);
               out += paramUDig << (i - 1);
            }
         }//For

         //Update records
         if (!digInitialUpdate_) {
            //Forced callbacks even if no value change
            //ValueMask = 0xFF because a byte is 8 bits
            //Database input records are arranged by byte
            setUIntDigitalParam(0, GalilBinaryIn_, in, 0xFF, 0xFF );
            //Database output records are arranged by word
            //ValueMask = 0xFFFF because a word is 16 bits
            setUIntDigitalParam(0, GalilBinaryOutRBV_, out, 0xFFFF, 0xFFFF );
         }
         else {
            //Callbacks happen on value change
            setUIntDigitalParam(0, GalilBinaryIn_, in, 0xFF );
            setUIntDigitalParam(0, GalilBinaryOutRBV_, out, 0xFFFF );
         }

      }//DMC3000 model
      else {
         //for all models except DMC30000 series
         //digital inputs in banks of 8 bits for all models except DMC30000 series
         //Specify source 1, digital input byte
         strcpy(src1, "_TIx");
         for (addr=0;addr<BINARYIN_BYTES;addr++) {
            //Specify byte
            src1[3] = addr + ZEROASCII;
            paramUDig = (unsigned)sourceValue(recdata_, src1);
            //ValueMask = 0xFF because a byte is 8 bits
            if (!digInitialUpdate_) {
               //Forced callbacks even if no value change
               setUIntDigitalParam(addr, GalilBinaryIn_, paramUDig, 0xFF, 0xFF );
            }
            else {
               //Callbacks happen on value change
               setUIntDigitalParam(addr, GalilBinaryIn_, paramUDig, 0xFF );
            }
         }//For
	 //data record has digital outputs in banks of 16 bits for dmc, 8 bits for rio
         //Specify source 1, digital output byte
         strcpy(src1, "_OPx");
	 for (addr=0;addr<BINARYOUT_WORDS;addr++) {
	    //Specify byte
            src1[3] = addr + ZEROASCII;
            paramUDig = (unsigned)sourceValue(recdata_, src1);
            if (!digInitialUpdate_) {
               //Forced callbacks even if no value change
               setUIntDigitalParam(addr, GalilBinaryOutRBV_, paramUDig, 0xFFFF, 0xFFFF );
            }
            else {
               //ValueMask = 0xFFFF because a word is 16 bits
               //Callbacks happen on value change
               setUIntDigitalParam(addr, GalilBinaryOutRBV_, paramUDig, 0xFFFF );
            }
         }//For 
      } //All models except DMC30000

      //Initial digital update completed, set flag so future record updates are on change only
      digInitialUpdate_ = true;

      //Analog ports
      //Port numbering is different on DMC compared to RIO controllers
      start = (rio_) ? 0 : 1;
      end = ANALOG_PORTS + start;

      //Specify source 1, analog input
      strcpy(src1, "@AN[x]");
      //Specify source 2, analog output readback
      strcpy(src2, "@AO[x]");
      for (addr = start;addr < end;addr++) {
         //Specify analog input number
         src1[4] = addr + ZEROASCII;
         paramDouble = (double)sourceValue(recdata_, src1);
         if ((paramDouble < (analogInPosted_[addr] - analogIndeadb_[addr])) || (paramDouble > (analogInPosted_[addr] + analogIndeadb_[addr]))) {
            setDoubleParam(addr, GalilAnalogIn_, paramDouble);
            analogInPosted_[addr] = paramDouble;
         }
         if (rio_ || model_[3] == '3') {
            //Specify analog output number
            src2[4] = addr + ZEROASCII;
            paramDouble = (double)sourceValue(recdata_, src2);
            if ((paramDouble < (analogOutRbvPosted_[addr] - analogOutRBVdeadb_[addr])) || (paramDouble > (analogOutRbvPosted_[addr] + analogOutRBVdeadb_[addr]))) {
               setDoubleParam(addr, GalilAnalogOutRBV_, paramDouble);
               analogOutRbvPosted_[addr] = paramDouble;
            }
         }//RIO or DMC30000
      }//For

      //Process unsolicited mesgs from controller
      processUnsolicitedMesgs();

      //Coordinate system status
      //Specify source 1, coordinate system move status
      strcpy(src1, "_BGx");
      //Specify source 2, coordinate system segment count
      strcpy(src2, "_CSx");
      for (addr=0;addr<COORDINATE_SYSTEMS;addr++) {
         //Specify coordinate system S or T to acquire move status from
         src1[3] = (addr) ? 'T' : 'S';
         setIntegerParam(addr, GalilCoordSysMoving_, (int)sourceValue(recdata_, src1));
         //Specify coordinate system S or T to acquire segment count from
         src2[3] = (addr) ? 'T' : 'S';
         setIntegerParam(addr, GalilCoordSysSegments_, (int)sourceValue(recdata_, src2));
      } //For

      //40xx and 41xx series, obtain internal amplifier fault status
      if (model_[3] == '4' && (model_[4] == '0' || model_[4] == '1')) {
         //Set the AD amplifier overcurrent status
         setIntegerParam(0, GalilAmpOverCurrentStatus_, (int)sourceValue(recdata_, "TA00"));
         //Set the AD amplifier overvoltage status
         setIntegerParam(0, GalilAmpOverVoltageStatus_, (int)sourceValue(recdata_, "TA01"));
         //Set the AD amplifier overtemperature status
         setIntegerParam(0, GalilAmpOverTemperatureStatus_, (int)sourceValue(recdata_, "TA02"));
         //Set the AD amplifier overtemperature status
         setIntegerParam(0, GalilAmpUnderVoltageStatus_, (int)sourceValue(recdata_, "TA03"));
         //Set the AD amplifier overcurrent status
         setIntegerParam(1, GalilAmpOverCurrentStatus_, (int)sourceValue(recdata_, "TA04"));
         //Set the AD amplifier overvoltage status
         setIntegerParam(1, GalilAmpOverVoltageStatus_, (int)sourceValue(recdata_, "TA05"));
         //Set the AD amplifier overtemperature status
         setIntegerParam(1, GalilAmpOverTemperatureStatus_, (int)sourceValue(recdata_, "TA06"));
         //Set the AD amplifier overtemperature status
         setIntegerParam(1, GalilAmpUnderVoltageStatus_, (int)sourceValue(recdata_, "TA07"));
         //Set the AD amplifier ELO status
         setIntegerParam(0, GalilAmpELOStatus_, (int)sourceValue(recdata_, "TA3AD"));
         //Set the EH amplifier ELO status
         setIntegerParam(1, GalilAmpELOStatus_, (int)sourceValue(recdata_, "TA3EH"));
      }
   } //connected_
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
bool GalilController::isprintable(int c)
{
   if ((10 == c) || (13 == c) || (0 != isprint(c))) {
      //Character is printable
      return true;
   }
   //Character not printable
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
                 if (((input[i] & 0x80) == 0x80) && (isprintable((int)value)))
                    {
                    //Byte looks like an unsolicited packet
                    //Check for overrun
                    if (j > MAX_GALIL_STRING_SIZE - 2)
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
  if (consecutive_timeouts_ > ALLOWED_TIMEOUTS) {
     //Disconnect
     disconnect();
     //Due to error, put poller to sleep at end of this cycle
     poller_->sleepPoller(false);
  }

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
asynStatus GalilController::sync_writeReadController(bool testQuery, bool logCommand)
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

  /*asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s: controller=\"%s\" command=\"%s\"\n", functionName, address_, cmd_);*/

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
  /*asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
         "%s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      functionName, address_, cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));*/

  if (debug_file != NULL && logCommand)
     {
     time_t now;
     //Use line buffering, then flush
     setvbuf(debug_file, NULL, _IOLBF, BUFSIZ);
     time(&now);
     char time_buffer[64];
     strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
     fprintf(debug_file, "%s (%d) %s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      time_buffer, getpid(), functionName, address_.c_str(), cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));
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
  string inp = "";                          //Solicited data concatenated over multiple reads				
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
        status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_DATAREC_SIZE, nread, &eomReason);
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
              if (((buf[i] & 0x80) == 0x80) && (isprintable((int)value)))
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
           }
        else //Stop read if any asyn error
           return asynError;
        }//while (!done)
     //Copy solicited response into user supplied buffer
     strcpy(input, resp);
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

/** Writes a string to the Galil controller and reads a response.
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
asynStatus GalilController::programDownload(const string prog)
{
  string msg = "";
  string progLocal(prog); // Local copy of DMC prog
  size_t nwrite;	//Asyn number of bytes written
  size_t nread;		//Asyn number of bytes read
  asynStatus status = asynError;	//Asyn status
  char buf[MAX_GALIL_STRING_SIZE];	//Read back response controller gives at end
  int eomReason;	//end of message reason when reading
  // Only if connected
  if (connected_)
     {
     //Request download
     status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, "DL\r", 3, &nwrite);
     //Insert download terminate character at program end
     progLocal.push_back('\\');
     //Download the program
     if (!status)
        {
        status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncGalil_, progLocal.c_str(), progLocal.length(), &nwrite);
        if (!status)  //Read "::" response that controller gives
           status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, (char *)buf, 2, &nread, &eomReason);
        if (buf[0] == '?' || buf[1] == '?')
           status = asynError;  //Controller didn't like the program
        }
     }
  // Report any error
  if (asynSuccess != status) {
    //Download fail
    msg = "Error downloading code model " + model_ + ", address " + address_;
    setCtrlError(msg);
  }
  // Return result
  return status;
}

/** Validate DMC program
  * \param[in] output Pointer to the output string.
  * \return asynStatus
  */
asynStatus GalilController::programValidate(const string prog) {
  string msg = "";
  string progLine;
  unsigned lineNo = 0;
  stringstream progStream(prog);
  while (getline(progStream, progLine, '\r')) {
    if (progLine.size() > 80) {
      msg = "Error DMC program line " + tsp(lineNo) + " length = ";
      msg += tsp(progLine.size()) + " > maximum @ 80 characters";
      setCtrlError(msg);
      // Return error
      return asynError;
    }
    lineNo++;
  }
  // Return success
  return asynSuccess;
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
  char buf[MAX_GALIL_DATAREC_SIZE];//Array upload raw data
  string mesg = "";		//Controller message
  char arrayName[MAX_GALIL_STRING_SIZE - 6];//Array name on controller to upload
                                            //Array upload command is 6 chars without array name
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
        strcpy(cmd, "QU ");
        strncat(cmd, arrayName, (size_t)(MAX_GALIL_STRING_SIZE - 6));
        strcat(cmd, "[]\r");
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
              status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_DATAREC_SIZE, &nread, &eomReason);
              if (!status && nread > 0)
                 {
                 //Search for error and terminating characters
                 for (i = 0; i < nread; i++)
                    {
                    if (buf[i] == '?')
                       {
                       //Error
                       done = true;
                       mesg = "Array " + tsp(j,0) + " upload failed, array name " + tsp(arrayName) + " is unknown";
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
  char buf[MAX_GALIL_DATAREC_SIZE];

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
           status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncGalil_, buf, MAX_GALIL_DATAREC_SIZE, &nread, &eomReason);
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

   //Controller wide variables
   //Set tcperr counter to 0
   sprintf(cmd_, "tcperr=0");
   status = sync_writeReadController();

   //Set cmderr counter to 0
   sprintf(cmd_, "cmderr=0");
   status |= sync_writeReadController();

   //Activate input interrupts for motor interlock function
   if (!rio_) {
      if (digitalinput_init_ && digports_)
         sprintf(cmd_, "dpon=%d;dvalues=%d;mlock=1", digports_, digvalues_);
      else
         sprintf(cmd_, "dpon=%d;dvalues=%d;mlock=0", digports_, digvalues_);
      sync_writeReadController();
   }

   //Before burning variables backup the commutation initialized, and homed status flags
   //Then set status flags to 0 ready for burn to eeprom
   //Done so commutation initialized and homed status is always 0 at controller power on
   //Finally set axis variables
   if (numAxes_ > 0 && !status && !rio_)
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
         //provide sensible default for normdc (normal deceleration) value
         sprintf(cmd_, "nrmdc%c=%ld", axisList_[i], maxAcceleration_);
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
         //Initialize home switch edge
         sprintf(cmd_, "hswedg=1");
         status |= sync_writeReadController();
         //Initialize use limits as home switch
         sprintf(cmd_, "ulah%c=1", axisList_[i]);
         status |= sync_writeReadController();
         //Initialize home jog speed
         sprintf(cmd_, "hjs%c=2048", axisList_[i]);
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
         errlogPrintf("Error burning variables to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());
         }
      else
         errlogPrintf("Burning variables to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());
      }

   //Restore previous homed, and commutation initialized status
   if (numAxes_ > 0 && !rio_)
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
      errlogPrintf("Error setting variables on model %s, address %s\n",model_.c_str(), address_.c_str());

   //Return status
   return status;
}

/** Replace generated axis home code with user code 
  * \param[in] axis Axis home code to replace
  * \param[in] filename User provided home code */
void GalilController::GalilReplaceHomeCode(char *axis, string filename) {
   GalilAxis *pAxis;              //GalilAxis
   char axisName;                 //Axis name specified
   string str;                    //Search string.  Home section start string (eg. IF(HOMEA=1))	
   string line;                   //Read the file line by line
   string mesg = "";              //Error messages
   ifstream file(filename);       //Input file stream
   size_t codeStart;              //Location of home code start in thread code
   size_t codeLen;                //Length of home code found
   size_t endLen;                 //Length of home code end statement
   size_t inpos;                  //Position in code to insert custom code line

   // Axis letter A-H
   axisName = (char)(toupper(axis[0]));
   axis[0] = axisName;
   axis[1] = '\0';

   // Check specified axis
   if (axisName < 65 || axisName > 72 ) {
      mesg = "GalilReplaceHomeCode: Bad axis specified, must be A-H";
      setCtrlError(mesg);
      return;
   }

   // Retrieve the specified axis instance
   pAxis = getAxis(axisName - AASCII);

   // Check specified axis
   if (!pAxis) {
      mesg = "GalilReplaceHomeCode: Specified axis not found";
      setCtrlError(mesg);
      return;
   }

   // Check if code is already assembled
   if (code_assembled_) {
      mesg = "GalilReplaceHomeCode: Should be called before GalilStartController";
      setCtrlError(mesg);
      return;
   }

   // Look for home program start in thread code
   str = string("IF(home") + string(axis) + string("=1)");
   codeStart = thread_code_.find(str);

   // Look for home program end in thread code
   str = "WT10;hjog?=0;home?=0;homed?=1;MG \"homed?\",homed?;ENDIF;ENDIF\n";
   endLen = str.length();
   //Replace ? symbol with axisName_
   replace( str.begin(), str.end(), '?', axisName);
   codeLen = thread_code_.find(str);

   if (codeStart != string::npos && codeLen != string::npos) {
      // Generated home program found
      if (file.is_open()) {
         // Calculate home code length
         codeLen += endLen;
         codeLen -= codeStart;
         // Remove generated home code
         thread_code_.erase(codeStart, codeLen);
         // Initial position within code buffer to insert custom code
         inpos = codeStart;
         // Read provided file line by line, and add to code buffer
         while (getline(file, line)) {
            //Skip REM lines
            if (line.find("REM") != string::npos) {
               continue;
            }         
            // Put the new line character back
            line = line + '\n';
            //Replace empty lines with ' apostrophe
            if (line.compare("\n") == 0) {
               line = "'\n";
            }
            else {
               // Check read line for $(AXIS) macro
               while ((codeStart = line.find("$(AXIS)")) != string::npos) {
                  // Replace any found $(AXIS) macro with specified axis
                  if (codeStart != string::npos) {
                     line.replace(codeStart, 7, axis);
                  }
               }
            }
            // Add custom code line to code
            thread_code_.insert(inpos, line);
            // Increment inpos to next insert position
            inpos = inpos + line.length();
         }
         // Done reading, close the file
         file.close();
         // Flag custom home code loaded for this axis
         pAxis->customHome_ = true;
      }
      else {
        // Can't open provided file
        mesg = "GalilReplaceHomeCode: Couldn't open " + filename;
        setCtrlError(mesg);
      }
   }
   else {
      mesg = "GalilReplaceHomeCode: Can't find generated home prog for axis " + string(axis);
      setCtrlError(mesg);
      return;
   }
}

/** Adds user custom code to generated code
  * \param[in] section Code section to put user code
                       0 = Card code
                       1 = Thread code
                       2 = Limits code
                       3 = Digital code
  * \param[in] filename Code file specified by user */
void GalilController::GalilAddCode(int section, string filename) {
   size_t found = string::npos;     //Location of search string in thread code
   char axis = 'A';                 //Axis associated with specified section of thread code
   string line;                     //Read the file line by line
   string mesg = "";                //Error messages
   ifstream file(filename);         //Custom code to add
   size_t inpos = string::npos;     //Position in code to insert custom code line

   if (section == 3 && !digitalinput_init_) {
      mesg = "GalilAddCode: Digital section not defined, review GalilCreateAxis";
      setCtrlError(mesg);
      return;
   }

   // Check if code is already assembled
   if (code_assembled_) {
      mesg = "GalilAddCode: Should be called before GalilStartController";
      setCtrlError(mesg);
      return;
   }

   if (file.is_open()) {
      //File open okay

      //Thread code
      if (section == 1) {
         //Check for jump statement at end of existing thread_code_
         found = thread_code_.find("JP #THREAD", thread_code_.size() - 12);
         if (found != string::npos) {
            //Jump statement found
            //Store axis associated with this part of the thread code
            axis = thread_code_[found + 10];
            //Initial position within thread code to insert custom code
            inpos = found;
         }
      }

      //Read provided file, add to specified code buffer
      while (getline(file, line)) {
         //Skip REM lines
         if (line.find("REM") != string::npos) {
            continue;
         }         
         // Put the new line character back
         line = line + '\n';
         //Replace empty lines with ' apostrophe
         if (line.compare("\n") == 0) {
            line = "'\n";
         }
         //Add line to specified code section
         if (!section)
            card_code_ += line;
         if (section == 2)
            limit_code_ += line;
         if (section == 3)
            digital_code_ += line;
         if (section == 1 && inpos != string::npos) {
            // Check read line for $(AXIS) macro
            while ((found = line.find("$(AXIS)")) != string::npos) {
               // Replace any found $(AXIS) macro with found axis
               if (found != string::npos)
                  line.replace(found, 7, string(1, axis));
            }
            //Add line to specified code section
            thread_code_.insert(inpos, line);
            //Increment inpos to next insert position
            inpos = inpos + line.length();
         }
      }
      // Done reading, close the file
      file.close();
   }
   else {
      // Can't open provided file
      mesg = "GalilAddCode: Cannot open " + filename;
      setCtrlError(mesg);
   }
}

/*--------------------------------------------------------------*/
/* Start the card requested by user   */
/*--------------------------------------------------------------*/
void GalilController::GalilStartController(char *code_file, int burn_program, int thread_mask)
{
   int status;			//Status
   string mesg = "";		//Error messages
   GalilAxis *pAxis;		//GalilAxis
   unsigned i;			//General purpose looping
   bool start_ok = true;	//Have the controller threads started ok
   downloadState downloadStatus = notRequired;	//Code download status default
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
   if (!code_assembled_) {
      //Generate rio code if required
      gen_rio_code();
      //Assemble the code generated, if we havent already
      //First add termination code to end of code generated for this card
      gen_card_codeend();

      //Assemble the sections of generated code for this card
      card_code_ += thread_code_;
      card_code_ += limit_code_;
      card_code_ += digital_code_;
      //Dump generated codefile, which we may or may not actually use
      write_gen_codefile("_gen");

      //load up code file specified by user, if any
      if (!read_codefile(code_file)) {
         //Copy the user code into card code buffer
         //Ready for delivery to controller
         card_code_ = user_code_;
         //Set customHome flag for all axis
         for (i = 0; i < numAxes_; i++) {
            pAxis = getAxis(axisList_[i] - AASCII);
            if (!pAxis) continue;
            pAxis->customHome_ = true;
         }
      }
      else
         thread_mask_ = (rio_) ? thread_mask_ : 0;  //DMC forced to use generated code

      //Dump card_code_ to file
      write_gen_codefile("_prd");
   }

   //Print out the generated/user code for the controller
   if ((display_code == 1) || (display_code == 3)) {
      printf("\nGenerated/User code is\n\n");
      cout << card_code_ << endl;
   }

   //If connected, then proceed
   //to check the program on dmc, and download if needed
   if (connected_) {
      //Increase timeout whilst manipulating controller code
      timeout_ = 10;
      //Upload code currently in controller for comparison to generated/user code
      status = programUpload(&uc);
      if (status) //Upload failed
         errlogPrintf("\nError uploading code model %s, address %s\n",model_.c_str(), address_.c_str());

      if ((display_code == 2) || (display_code == 3)) {
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
      if (dc.compare(uc) != 0 && dc.compare("") != 0) {
         if (asynSuccess == programValidate(dc)) {
           // Program validated successfully
           errlogPrintf("\nTransferring code to model %s, address %s\n",model_.c_str(), address_.c_str());		
           //Do the download
           if (asynSuccess == programDownload(dc)) {
             //Download success
             downloadStatus = downloadSuccess;
           }
           else {
             //Download fail
             downloadStatus = downloadFailed;
           }
         }
         else {
           // Program validation failed
           downloadStatus = downloadFailed;
         }
	
         if (downloadSuccess == downloadStatus) {
            errlogPrintf("Code transfer successful to model %s, address %s\n",model_.c_str(), address_.c_str());	
            //Burn program code to eeprom if burn_program is 1
            if (burn_program == 1) {
               //Burn program to EEPROM
               sprintf(cmd_, "BP");
               if (sync_writeReadController() != asynSuccess)
                  errlogPrintf("Error burning code to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());
               else
                  errlogPrintf("Burning code to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());

               //Burn parameters to EEPROM
               sprintf(cmd_, "BN");
               if (sync_writeReadController() != asynSuccess)
                  errlogPrintf("Error burning parameters to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());
               else
                  errlogPrintf("Burning parameters to EEPROM model %s, address %s\n",model_.c_str(), address_.c_str());
            } //if burn_program
         } //download ok
      } //Code on controller different

      if (downloadFailed != downloadStatus) {
         //Initialize galil code variables
         burn_variables = (burn_program && (downloadSuccess == downloadStatus)) ? true : false;
         variables_ok = GalilInitializeVariables(burn_variables) == asynSuccess ? true : false;

         /*Upload code currently in controller to see whats there now*/              
         if (asynSuccess == programUpload(&uc)) {
            //Remove the \r characters - \r\n is returned by galil controller
            uc.erase (std::remove(uc.begin(), uc.end(), '\r'), uc.end());
            //Change \n to \r (Galil Communications Library expects \r separated lines)
            std::replace(uc.begin(), uc.end(), '\n', '\r');
            //Some controllers dont finish upload with \r\n, ensure buffer ends in carriage return
            if (uc.back() != 13)
               uc.push_back('\r');
         }
         else {
            errlogPrintf("\nError uploading code model %s, address %s\n",model_.c_str(), address_.c_str());
         }

         //Start thread 0 if code on controller matches what was downloaded
         //Its assumed that thread 0 starts any other required threads on controller
         if ((0 == (int)dc.compare(uc)) && 0 != (int)uc.compare("") && thread_mask_ >= 0) {
            //Start thread 0
            sprintf(cmd_, "XQ 0,0");
            if (sync_writeReadController() != asynSuccess)
               errlogPrintf("Thread 0 failed to start on model %s address %s\n\n",model_.c_str(), address_.c_str());

            //Wait a second before checking thread status
            epicsThreadSleep(1);
			
            //Check threads on controller
            if (thread_mask_ > 0) {
               //user specified a thread mask, only check for these threads
               for (i = 0; i < 32; ++i) {
                  if ( (thread_mask_ & (1 << i)) != 0 ) {
                     //Check that code is running
                     sprintf(cmd_, "MG _XQ%d", i);
                     if (sync_writeReadController() == asynSuccess) {
                        if (atoi(resp_) == -1) {
                           start_ok = false;
                           mesg = "Thread " + tsp(i, 0) + " failed to start on model " + model_ + ", address " + address_;
                           setCtrlError(mesg);
                        }
                     }
                  }
               }
            }
            else if ((numAxes_ > 0 || rio_) && thread_mask_ == 0) {
               //Check code is running for all created GalilAxis
               if (rio_) //Check thread 0 on rio
                  numAxes_ = 1;
               for (i = 0;i < numAxes_;i++) {		
                  //Check that code is running
                  if (rio_)
                     sprintf(cmd_, "MG _XQ%d", i);
                  else
                     sprintf(cmd_, "MG _XQ%d", axisList_[i] - AASCII);
                  if (sync_writeReadController() == asynSuccess) {
                     if (atoi(resp_) == -1) {
                        start_ok = false;
                        if (rio_)
                           mesg = "Thread " + tsp(i, 0) + " failed to start on model " + model_ + ", address " + address_;
                        else
                           mesg = "Thread " + tsp(axisList_[i] - AASCII, 0) + " failed to start on model " + model_ + ", address " + address_;
                        setCtrlError(mesg);
                     }
                  }
               }
            }

	    if (start_ok)
               errlogPrintf("Code started successfully on model %s, address %s\n",model_.c_str(), address_.c_str());
         } // Start thread 0
      }
      else // Download failed
         start_ok = false;

      //Limits motor direction consistency unknown at connect/reconnect
      for (i = 0; i < numAxes_; i++) {
         pAxis = getAxis(axisList_[i] - AASCII);
         if (!pAxis) continue;
         pAxis->limitsDirState_ = unknown;
         //Pass motor/limits consistency to paramList
         setIntegerParam(pAxis->axisNo_, GalilLimitConsistent_, pAxis->limitsDirState_);
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
   if (start_ok && variables_ok && (downloadStatus == downloadSuccess || downloadStatus == notRequired))
      start_ok = true;
   else
      start_ok = false;
   setIntegerParam(GalilStart_, (int)start_ok);

   //Code is assembled.  Free RAM, and set flag accordingly
   //Keep card_code_ for re-connection/re-start capability
   //Keep card_code_ so we can call GalilStartController internally for re-start
   if (!code_assembled_) {
      //free RAM
      thread_code_.clear();
      limit_code_.clear();
      digital_code_.clear();
      user_code_.clear();
      //The GalilController code is fully assembled, and stored in GalilController::card_code_
      code_assembled_ = true;
   }
}

//Generate code headers that are always required for both dmc and rio
void GalilController::gen_code_headers(void)
{
   //setup #AUTO label
   card_code_ = "#AUTO\n";
}

/* Generate code for rio */
void GalilController::gen_rio_code(void) {
   //RIO thread code
   if (rio_ || numAxes_ == 0) {
      thread_code_ += "#THREADA\nWT 500\nJP #THREADA\n";
   }
}

/*--------------------------------------------------------------------------------*/
/* Generate code end, and controller wide code eg. error recovery*/

void GalilController::gen_card_codeend(void)
{
   unsigned i;
   struct Galilmotor_enables *motor_enables = NULL;  //Convenience pointer to GalilController motor_enables[digport]

   //Calculate the digports and digvalues required for motor interlock function
   for (i = 0; i < 8; i++) {
      //Retrieve structure for digital port from controller instance
      motor_enables = (Galilmotor_enables *)&motor_enables_[i];
      //If this digital in port has at least 1 motor, include it
      if (strlen(motor_enables->motors) > 0) {
         //Include the port
         digports_ = digports_ | (1 << i);
         //Include the disable value 
         digvalues_ = digvalues_ | (motor_enables->disablestates[0] << i);
      }
   } //For

   //generate code end, only if axis are defined
   if (numAxes_ != 0) {
      // Add galil program termination code
      if (digitalinput_init_ == true) {
         //Limit code has been written, and we are done with LIMSWI but its not prog end
         limit_code_ += "RE 1\n";
         //Add controller wide motor interlock code to #ININT
         if (digports_ != 0)
            gen_motor_enables_code();
	
         //Add code to end digital port interrupt routine, and end the prog
         digital_code_ += "RI 1\nEN\n";
      }
      else  //Limit code has been written, and we are done with LIMSWI and its prog end
         limit_code_ += "RE 1\nEN\n";
   }

   //Add command error handler
   thread_code_ += "#CMDERR\nerrstr=_ED;errcde=_TC;cmderr=cmderr+1\nEN1\n";

   //Add tcp error handler
   thread_code_ += "#TCPERR\ntcpcde=_TC;tcperr=tcperr+1\nRE1\n";

   if (rio_ || numAxes_ == 0) {
      limit_code_ += "EN\n";
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
  for (i = 0; i < 8; i++) {
     //Retrieve structure for digital port from controller instance
     motor_enables = (Galilmotor_enables *)&motor_enables_[i];
     //Generate if statement for this digital port
     if (strlen(motor_enables->motors) > 0) {
        any = true;
        digital_code_ += "IF(@IN[" + tsp(i + 1, 0) + "]=" + tsp((int)motor_enables->disablestates[0], 0);
        digital_code_ += ")\n";
        //Scan through all motors associated with the port
        for (j = 0; j<(int)strlen(motor_enables->motors); j++) {
           c = motor_enables->motors[j];
           //Add code to stop the motors when digital input state matches that specified
           if (((j != 0) && ((j % 4) == 0)) || (j == ((int)strlen(motor_enables->motors) - 1))) {
              digital_code_ += "ST" + tsp(c) + ";DC" + tsp(c) + "=limdc" + tsp(c) + "\n";
           }
           else {
              digital_code_ += "ST" + tsp(c) + ";DC" + tsp(c) + "=limdc" + tsp(c) + ";";
           }
        } //For
        //Manipulate interrupt flag to turn off the interrupt on this port for one threadA cycle
        digital_code_ += "dpoff=dpoff-" + tsp((1 << i),0) + ";ENDIF\n";
     }
  } //For

  /* Re-enable input interrupt for all except the digital port(s) just serviced during interrupt routine*/
  /* ThreadA will re-enable the interrupt for the serviced digital ports(s) after 1 cycle */
  if (any)
     digital_code_ += "II ,,dpoff,dvalues\n";
}

/*-----------------------------------------------------------------------------------*/
/*  Dump galil code generated for this controller to file
*/

void GalilController::write_gen_codefile(const char* suffix)
{
	FILE *fp;
	int i = 0;
	char filename[100];
	
	sprintf(filename,"./%s%s.dmc",address_.c_str(), suffix);
	
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
		free(code_file_copy);
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
  string line;
  string user_code = "";
  int status = asynSuccess;
  
  //local temp code buffers
  int max_size = MAX_GALIL_AXES * (THREAD_CODE_LEN+LIMIT_CODE_LEN+INP_CODE_LEN);
  char* user_code_exp = (char*)calloc(max_size,sizeof(char));

  if (strcmp(code_file, "") != 0) {
    ifstream file(code_file);  //Input file stream
    // Test if file opened
    if (file.is_open()) {
      // Read provided file line by line
      while (getline(file, line)) {
        // Skip REM lines
        if (line.find("REM") != string::npos) {
          continue;
        }
        // Put the new line character back
        line = line + '\n';
        //Replace empty lines with ' apostrophe
        if (line.compare("\n") == 0) {
          line = "'\n";
        }
        // Add code line to user_code buffer
        user_code += line;
      }
      // Done reading, close the file
      file.close();
      //Load galil code into the GalilController instance
      if (mac_handle != NULL) {// substitute macro definitios for e.g. $(AXIS)
        macExpandString(mac_handle, user_code.c_str(), user_code_exp, max_size);
        //Copy code into GalilController temporary area
        user_code_ += user_code_exp;
      }
      else {
        //Copy code into GalilController temporary area
        user_code_ += user_code;
      }
    }
    else {
      if (rio_) {
        errlogPrintf("\nread_codefile_part: Can't open user code file \"%s\"\n\n", code_file);
      }
      else {
        errlogPrintf("\nread_codefile_part: Can't open user code file \"%s\", using generated code\n\n", code_file);
      }  
      // Can't open specified file      
     status = asynError;
    }
  }
  // Free user_code_exp
  free(user_code_exp);
  // Return status
  return (asynStatus)status;
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
void GalilController::setCtrlError(string mesg)
{
   if (mesg.size() > 0)
      std::cout << mesg << std::endl;
   setStringParam(0, GalilCtrlError_, mesg.c_str());
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
	map["TA3AD"] = Source(52, "UB", 0, "Boolean", "Axis A-D ELO active");
	map["TA3EH"] = Source(52, "UB", 1, "Boolean", "Axis E-H ELO active");

	map["TA2A"] = Source(53, "UB", 0, "Boolean", "Axis A at _TKA peak current");
	map["TA2B"] = Source(53, "UB", 1, "Boolean", "Axis B at _TKB peak current");
	map["TA2C"] = Source(53, "UB", 2, "Boolean", "Axis C at _TVC peak current");
	map["TA2D"] = Source(53, "UB", 3, "Boolean", "Axis D at _TKD peak current");
	map["TA2E"] = Source(53, "UB", 4, "Boolean", "Axis E at _TKE peak current");
	map["TA2F"] = Source(53, "UB", 5, "Boolean", "Axis F at _TKF peak current");
	map["TA2G"] = Source(53, "UB", 6, "Boolean", "Axis G at _TKG peak current");
	map["TA2H"] = Source(53, "UB", 7, "Boolean", "Axis H at _TKH peak current");

	map["TA1A"] = Source(54, "UB", 0, "Boolean", "Axis A hall error");
	map["TA1B"] = Source(54, "UB", 1, "Boolean", "Axis B hall error");
	map["TA1C"] = Source(54, "UB", 2, "Boolean", "Axis C hall error");
	map["TA1D"] = Source(54, "UB", 3, "Boolean", "Axis D hall error");
	map["TA1E"] = Source(54, "UB", 4, "Boolean", "Axis E hall error");
	map["TA1F"] = Source(54, "UB", 5, "Boolean", "Axis F hall error");
	map["TA1G"] = Source(54, "UB", 6, "Boolean", "Axis G hall error");
	map["TA1H"] = Source(54, "UB", 7, "Boolean", "Axis H hall error");

	map["TA00"] = Source(55, "UB", 0, "Boolean", "Axis A-D over current");
	map["TA01"] = Source(55, "UB", 1, "Boolean", "Axis A-D over voltage");
	map["TA02"] = Source(55, "UB", 2, "Boolean", "Axis A-D over temperature");
	map["TA03"] = Source(55, "UB", 3, "Boolean", "Axis A-D under voltage");
	map["TA04"] = Source(55, "UB", 4, "Boolean", "Axis E-H over current");
	map["TA05"] = Source(55, "UB", 5, "Boolean", "Axis E-H over voltage");
	map["TA06"] = Source(55, "UB", 6, "Boolean", "Axis E-H over temperature");
	map["TA07"] = Source(55, "UB", 7, "Boolean", "Axis E-H under voltage");

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
  * \param[in] enables_string	 Comma separated list of digital IO ports used to enable/disable the motor
  * \param[in] switch_type	 Switch type attached to digital bits for enable/disable motor
  */
extern "C" asynStatus GalilCreateAxis(const char *portName,        	/*specify which controller by port name */
                         	      char *axisname,                  	/*axis name A-H */
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

  new GalilAxis(pC, axisname, enables_string, switch_type);

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

/** Replace generated home code with custom home code
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  * \param[in] axis      	 Which axis is the provided home program intended for
  * \param[in] code_file      	 Code file to add to generated code
  */
extern "C" asynStatus GalilReplaceHomeCode(const char *portName,        	//specify which controller by port name
					   char *axis,
					   const char *code_file)
{
  GalilController *pC;
  static const char *functionName = "GalilReplaceHomeCode";

  //Convert provided code file into string
  string filename(code_file);

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, portName);
    return asynError;
  }
  pC->lock();
  //Call GalilController::GalilReplaceHomeCode to do the work
  pC->GalilReplaceHomeCode(axis, filename);
  pC->unlock();
  return asynSuccess;
}

/** Add custom code to generated code
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  * \param[in] code_file      	 Code file to add to generated code
  * \param[in] section      	 Where to add custom code
  */
extern "C" asynStatus GalilAddCode(const char *portName,        	//specify which controller by port name
					int section,
					const char *code_file)
{
  GalilController *pC;
  static const char *functionName = "GalilAddCode";

  //Convert provided code file into string
  string filename(code_file);

  //Retrieve the asynPort specified
  pC = (GalilController*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n",
           driverName, functionName, portName);
    return asynError;
  }
  pC->lock();
  //Call GalilController::GalilAddCode to do the work
  pC->GalilAddCode(section, filename);
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
static const iocshArg GalilCreateAxisArg2 = {"Motor enable string", iocshArgString};
static const iocshArg GalilCreateAxisArg3 = {"Motor enable switch type", iocshArgInt};

static const iocshArg * const GalilCreateAxisArgs[] =  {&GalilCreateAxisArg0,
                                                        &GalilCreateAxisArg1,
							&GalilCreateAxisArg2,
							&GalilCreateAxisArg3};

static const iocshFuncDef GalilCreateAxisDef = {"GalilCreateAxis", 4, GalilCreateAxisArgs};

static void GalilCreateAxisCallFunc(const iocshArgBuf *args)
{
  GalilCreateAxis(args[0].sval, args[1].sval, args[2].sval, args[3].ival);
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

//GalilReplaceHomeCode iocsh function
static const iocshArg GalilReplaceHomeCodeArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilReplaceHomeCodeArg1 = {"Axis", iocshArgString};
static const iocshArg GalilReplaceHomeCodeArg2 = {"Code file", iocshArgString};
static const iocshArg * const GalilReplaceHomeCodeArgs[] = {&GalilReplaceHomeCodeArg0,
                                                    &GalilReplaceHomeCodeArg1,
                                                    &GalilReplaceHomeCodeArg2};
                                                             
static const iocshFuncDef GalilReplaceHomeCodeDef = {"GalilReplaceHomeCode", 3, GalilReplaceHomeCodeArgs};

static void GalilReplaceHomeCodeCallFunc(const iocshArgBuf *args)
{
  GalilReplaceHomeCode(args[0].sval, args[1].sval, args[2].sval);
}

//GalilAddCode iocsh function
static const iocshArg GalilAddCodeArg0 = {"Controller Port name", iocshArgString};
static const iocshArg GalilAddCodeArg1 = {"Section", iocshArgInt};
static const iocshArg GalilAddCodeArg2 = {"Code file", iocshArgString};
static const iocshArg * const GalilAddCodeArgs[] = {&GalilAddCodeArg0,
                                                    &GalilAddCodeArg1,
                                                    &GalilAddCodeArg2};
                                                             
static const iocshFuncDef GalilAddCodeDef = {"GalilAddCode", 3, GalilAddCodeArgs};

static void GalilAddCodeCallFunc(const iocshArgBuf *args)
{
  GalilAddCode(args[0].sval, args[1].ival, args[2].sval);
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
  iocshRegister(&GalilAddCodeDef, GalilAddCodeCallFunc);
  iocshRegister(&GalilReplaceHomeCodeDef, GalilReplaceHomeCodeCallFunc);
  iocshRegister(&GalilStartControllerDef, GalilStartControllerCallFunc);
}

//Finally do the registration
extern "C" {
epicsExportRegistrar(GalilSupportRegister);
}

