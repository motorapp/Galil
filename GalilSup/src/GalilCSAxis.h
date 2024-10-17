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

#ifndef GalilCSAxis_H
#define GalilCSAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

class GalilCSAxis : public asynMotorAxis
{
public:

  GalilCSAxis(class GalilController *pC, 	//The GalilController
		char axisname);			//The coordinate system axis name I-P

  //These are the methods that are new to this class
  //Poller for CSAxis
  asynStatus poller(void);
  //Store settings, and implement defaults
  asynStatus setDefaults(void);
  //Construct axes list from provided equation
  asynStatus obtainAxisList(char axis, char *equation, char *axes);
  //Substitute transform equation in place of motor name
  asynStatus substituteTransforms(char axis, char *equation);
  //Bring variables Q-X in range A-P for use with sCalcperform
  asynStatus substituteVariables(char axis, char *equation, char *axes, char *vars, char *subs);
  //Parse a kinematic transform equation and store results in GalilCSAxis instance
  asynStatus parseTransform(char axis, char *equation, char *axes, char *vars, char *subs);
  //Store kinematics when user changes them
  asynStatus parseTransforms(void);
  //Calculate an expression with the given arguments
  asynStatus doCalc(const char *expr, double args[], double *result);
  //Calculate real axis orientation relative to this CSAxis
  asynStatus calcAxisLimitOrientation(void);
  //Get forward axis setpoints (Related CSAxis), and pack into spargs
  asynStatus packSetPointArgs(double spargs[]);
  //Get axis position readbacks and pack into mrargs
  asynStatus packReadbackArgs(char *axes, double mrargs[]);
  //Peform forward kinematic transform using real axis readback data, and store results in GalilCSAxis
  asynStatus forwardTransform(void);
  //Perform reverse coordinate and velocity transform
  asynStatus reverseTransform(double pos, double vel, double accel, double npos[], double nvel[], double naccel[], bool useCSSetpoints = true, bool profileMsg = false);
  //Transform CSAxis profile into Axis profiles
  asynStatus transformCSAxisProfile(void);
  //Selects a free coordinate system S or T and returns coordsys number, or -1 if none free
  int selectFreeCoordinateSystem(void);
  //Uses vector mathematics to check requested real motor velocities
  asynStatus checkMotorVelocities(double npos[], double nvel[], double naccel[]);
  //Check motor enable interlock status of all reverse axis
  asynStatus beginCheck(const char *caller);
  //Check motor record status for this axis
  asynStatus checkMRSettings(const char *caller);
  //Enforce CSAxis completion order
  int enforceCSAxisCompletionOrder(int csmoving);
  //Check CSAxis revaxes limit switch given move request
  asynStatus checkLimits(const char *caller, double npos[]);
  //Check CSAxis revaxes soft limit given move request
  asynStatus checkSoftLimits(const char *caller, double npos[]);
  //Check CSAxis all parameters
  asynStatus checkAllSettings(const char *caller, double npos[], double nvel[], double naccel[], bool bCheck = true);
  //Monitor CSAxis move, stop if problem
  asynStatus monitorCSAxisMove(void);
  //Clear CSAxis move dynamics at move completion
  asynStatus clearCSAxisDynamics(void);
  //Set CSAxis move flags prior to move
  asynStatus setupCSAxisMove(bool moveVelocity);
  //Service slow and infrequent requests from poll thread to write to the controller
  //We do this in a separate thread so the poll thread is not slowed
  //Also poll thread doesnt have a lock and is not allowed to call writeReadController
  void pollServices(void);
  //Driver internal version of CSAxis stop, prevents backlash, retries till dmov
  asynStatus stopInternal(bool emergencyStop = true);
  //Driver to motor record stop.  Stops backlash, retries.
  asynStatus stopMotorRecord(void);
  //Thread to start CSAxis moves when moveDeferred set false
  void startDeferredMovesThread();
  //Thread to receive axis events as they occur (eg. start, stop)
  void eventMonitorThread();
  //Start CSAxis moves that are not deferred
  void startCSAxisMoveThread();
  //Send axis events
  void sendAxisEvents(bool inmotion);

  /* These are the methods we override from the base class */
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setPosition(double position);
  asynStatus initializeProfile(size_t maxProfilePoints);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);

  virtual ~GalilCSAxis();

private:
  GalilController *pC_;      		/**< Pointer to the asynMotorController to which this axis belongs.
                                	*   Abbreviated because it is used very frequently */
  char axisName_;			//The axis letter I-P
  bool axisReady_;			//Have motor record fields been pushed into driver
  bool lastaxisReady_;			//Show CS axis as moving until after axis ready so initial setpoint in mr matches the readback
  char *fwdaxes_;			//List of CS motors whose transforms should be treated as forward transforms
  char *forward_;			//forward kinematic transform used to calculate the coordinate system (cs) motor position
  char *fwdvars_;			//Forward kinematic variables List of Q-Z
  char *fwdsubs_;			//Forward kinematic substitutes List of A-P
  char *revaxes_;			//List of real motors axis whose transforms should be treated as reverse transforms
  char **reverse_;			//Reverse transforms to calculate each axis position in the coordinate system
  char **revvars_;			//Reverse kinematic variables List of Q-Z
  char **revsubs_;			//Reverse kinematic substitutes List of A-P
  bool stopInternal_;			//Flag indicates stop request is from driver not MR
  bool stoppedMR_;			//Indicates motor record for this axis has been stopped
  bool stopSent_;			//Has stop request been sent to pollServices
  int stop_reason_;			//Reason for requested stop
  bool kinematicsAltered_;              //Have the kinematics equations been altered
  limitsState limitOrientation_[MAX_GALIL_AXES];//Orientation of real axis limits relative to this CSAxis
  bool move_started_;			//CSAxis move started
  bool kinematic_error_reported_;	//Kinematic error has been reported to user
  int last_done_;			//Done status stored from previous poll cycle
  int done_;				//Done status
  double highLimit_;			//High soft limit
  double lowLimit_;			//Low soft limit
  bool moveVelocity_;			//Indicates a jog move in progress
  double motor_position_;		//aux encoder or step count register
  double encoder_position_;		//main encoder register
  double last_motor_position_;		//aux encoder or step count register stored from previous poll
  int direction_;			//Direction of CSAxis
  int deferredCoordsys_;		//Coordinate system 0 (S) or 1 (T)
  double deferredAcceleration_;		//Coordinate system acceleration
  double deferredVelocity_;		//Coordinate system velocity
  double deferredPosition_;		//Deferred move position
  int deferredRelative_;		//Deferred move is relative or absolute
  bool deferredMove_;			//Has a deferred move been set
  int deferredMode_;			//Deferred mode.  Sync start only, sync start and stop
  double setPoint_;			//CSAxis setpoint position
  int cshoming_;			//CSAxis homing status
  int last_cshoming_;			//CSAxis last homing status
  epicsMessageQueue pollRequest_;	//The service numbers poll would like done

  bool shuttingDown_;			//Ioc shutdown
  bool requestedEventSent_;		//Poller has sent the requested event
  double requestedTimeout_;		//Event timeout
  epicsEventId beginEvent_;		//Axis begin event
  epicsEventId requestedEvent_;		//Event requested
  epicsEventId eventMonitorStart_;	//Tell event monitor thread to start monitoring
  epicsEventId eventMonitorDone_;	//Event monitor signals done to other threads
  epicsEventWaitStatus eventResult_;	//Event received or timeout

  epicsEventId deferredMoveStart_;	//Event to start CSAxis moves when movesDeferred set false

friend class GalilController;
friend class GalilAxis;
};

#endif   // GalilCSAxis_H

