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
// Thread to acquire datarecord for a single GalilController
// We write our own because epicsEventWaitWithTimeout in asynMotorController::asynMotorPoller waits, we dont want that.
// Instead either Async speed determines poller frequency or simple epicsThreadSleep in Sync mode

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <epicsThread.h>

using namespace std; //cout ostringstream vector string

#include "GalilController.h"

GalilPoller::GalilPoller(GalilController *pcntrl)
   :  thread(*this,"GalilPoller",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityMax)
{
	//Store the GalilController we poll for
	pC_ = pcntrl;
	//Create poller sleep event
  	pollerSleepEventId_ = epicsEventMustCreate(epicsEventEmpty);
  	//Create poller wake event
  	pollerWakeEventId_ = epicsEventMustCreate(epicsEventEmpty);
	//Poller awake at start
	pollerSleep_ = false;
	//Start GalilPoller thread
	shutdownPoller_ = false;
	thread.start();
}

//GalilPoller thread
//Acquire data record for a single GalilController
//Replaces asynMotorController::asynMotorPoller
void GalilPoller::run(void)
{
  unsigned i;
  bool moving;		//Moving flag
  asynMotorAxis *pAxis; //Axis structure
  double time_taken;	//Time taken last polll cycle
  double sleep_time;	//Calculated time to sleep in synchronous mode

  //Read current time
  epicsTimeGetCurrent(&pollnowt_);
  epicsTimeGetCurrent(&polllastt_);

  //Loop until shutdown
  while (true) 
	{
	//Dont poll in sleep mode or when shutting down
	if (!pollerSleep_ && !shutdownPoller_)
		{
		//Wake mode
		//Poll only if connected
		if (pC_->connected_)
                   {
                   //Get the data record, update controller related information in GalilController, and ParamList.  callBacks not called
                   pC_->poll();
                   //Read current time
                   epicsTimeGetCurrent(&pollnowt_);
                   //Calculate cycle time
                   time_taken = epicsTimeDiffInSeconds(&pollnowt_, &polllastt_);
                   //Store current time for next cycle
                   polllastt_.secPastEpoch = pollnowt_.secPastEpoch;
                   polllastt_.nsec = pollnowt_.nsec;

                   //if (time_taken > 0.02)
                   //	{
                   //   printf("%s GalilPoller %2.4lfs\n", pC_->model_, time_taken);
                   //	}

                   //Update the GalilAxis status, using datarecord from GalilController
                   //Do callbacks for GalilController, GalilAxis records
                   //Do all ParamLists/axis whether user called GalilCreateAxis or not
                   //because analog/binary IO data are stored/organized in ParamList just same as axis data 
                   for (i=0; i<MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
                      {
                      if (i < MAX_GALIL_AXES)
                         {
                         //Retrieve GalilAxis instance i
                         pAxis = pC_->getAxis(i);
                         }
                      else
                         {
                         //Retrieve GalilCSAxis instance i
                         pAxis = pC_->getCSAxis(i);
                         }
					
                      //Tolerate null GalilAxis object pointers
                      if (!pAxis) //User did not call GalilCreateAxis for this axis number
                         {
                         //Ensure callbacks are called to update upper layer analog/binary
                         //records for first 8 banks only
                         //Cant call GalilAxis->poll
                         if (i < MAX_GALIL_AXES)
                            pC_->callParamCallbacks(i); 
                         }
                      else				
                         pAxis->poll(&moving);		//Update GalilAxis, and upper layers, using retrieved datarecord
							//Update records with analog/binary data
                      }
                   //No async, so wait updatePeriod_ rather than relying on async record delivery frequency
                   if (!pC_->async_records_)
                      {
                      //Adjust sleep time according to time_taken last poll cycle
                      sleep_time = pC_->updatePeriod_/1000.0 - time_taken;
                      //Must sleep in synchronous mode to release lock for other threads
                      sleep_time = (sleep_time < 0.000) ? 0.001 : sleep_time;
                      if (sleep_time >= 0.001)
                         epicsThreadSleep(sleep_time);
                      }
                   }
                else //Not connected so sleep a little
                   epicsThreadSleep(.1);
	  	}
	else if (pollerSleep_ && !shutdownPoller_)
		{
		//Sleep mode
		//Inform blocking thread poller has now entered sleep mode
		epicsEventSignal(pollerSleepEventId_);
		//Sleep until signalled
		epicsEventWait(pollerWakeEventId_);
		}

	//Kill loop as IOC is shuttingDown
	if (shutdownPoller_)  //Break from loop
		break;
	}//while
}

void GalilPoller::shutdownPoller()
{
   //Send poller to sleep, so we know where the thread is
   sleepPoller();
   //Tell poller to shutdown
   shutdownPoller_ = true;
   //Pause
   epicsThreadSleep(.01);
   //Wake poller and send it to shutdown
   wakePoller(false);
   //Wait till poller thread exits
   thread.exitWait();
}

GalilPoller::~GalilPoller()
{
	shutdownPoller();
}

//Put poller in sleep mode, and stop async records if needed
//Must be called without lock so sync poller can be put to sleep
void GalilPoller::sleepPoller(void)
{
	//Only if poller awake now
	if (!pollerSleep_)
		{
		//Tell poller to sleep
		pollerSleep_ = true;
		//Wait until GalilPoller is sleeping
		epicsEventWait(pollerSleepEventId_);
		//Tell controller to stop async record transmission
		if (pC_->async_records_ && pC_->connected_)
			{
			pC_->lock();
			strcpy(pC_->cmd_, "DR 0");
			pC_->sync_writeReadController();
			pC_->unlock();
			}
		}
}

//Wake poller and re-start async records if desired
void GalilPoller::wakePoller(bool restart_async)
{
	//Only if poller sleeping now
	if (pollerSleep_)
		{
		//Wake up poller
		pollerSleep_ = false;
		epicsEventSignal(pollerWakeEventId_);
		//Tell controller to re-start async record transmission
		if (pC_->async_records_ && pC_->connected_ && restart_async)
			{
			sprintf(pC_->cmd_, "DR %.0f, %d", pC_->updatePeriod_, pC_->udpHandle_ - AASCII);
			pC_->sync_writeReadController();
			}
		}
}
