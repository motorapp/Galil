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

// Thread to acquire datarecord for a single GalilController
// We write our own because epicsEventWaitWithTimeout in asynMotorController::asynMotorPoller calls sleep, we dont want that.
// Needed better performance

class GalilPoller: public epicsThreadRunable {
public:
  GalilPoller(GalilController *pcntrl);
  void wakePoller(bool restart_async = true);
  void sleepPoller(bool waitTillSleep = true);
  virtual void run();
  epicsThread thread;
  ~GalilPoller();

private:
  GalilController *pC_;			//The GalilController we poll for
  bool pollerSleep_;			//Tell poller to sleep
  epicsTimeStamp pollstartt_;		//Used to calculate sleep time in synchronous mode
  epicsTimeStamp pollendt_;		//Used to calculate sleep time in synchronous mode

  epicsEventId pollerSleepEventId_;    	//Poller sleep event
  epicsEventId pollerWakeEventId_;    	//Poller Wake event
  bool shutdownPoller_;
  void shutdownPoller();
};
