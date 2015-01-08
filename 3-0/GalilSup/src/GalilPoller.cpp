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

#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <Galil.h>
#include <epicsThread.h>

using namespace std; //cout ostringstream vector string

#include "GalilController.h"

GalilPoller::GalilPoller(GalilController *cntrl)
   :  thread(*this,"GalilPoller",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityLow)
{
	//Store the GalilController we poll for
	pCntrl_ = cntrl;
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
  bool moving;
  asynMotorAxis *pAxis;

  //Loop until shutdown
  while (true) 
	{
	//Dont poll in sleep mode or when shutting down
	if (!pollerSleep_ && !shutdownPoller_)
		{
		//Wake mode
		//Poll only if connected
		if (pCntrl_->gco_ != NULL)
			{
			//Get the data record, update controller related information in GalilController, and ParamList.  callBacks not called
			pCntrl_->poll();
			//Update the GalilAxis status, using datarecord from GalilController
			//Do callbacks for GalilController, GalilAxis records
			//Do all ParamLists/axis whether user called GalilCreateAxis or not
			//because analog/binary IO data are stored/organized in ParamList just same as axis data 
			for (i=0; i<MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++)
				{
				if (i < MAX_GALIL_AXES)
					{
					//Retrieve GalilAxis instance i
					pAxis = pCntrl_->getAxis(i);
					}
				else
					{
					//Retrieve GalilCSAxis instance i
					pAxis = pCntrl_->getCSAxis(i);
					}
					
				//Tolerate null GalilAxis object pointers
				if (!pAxis) //User did not call GalilCreateAxis for this axis number
					{
					//Ensure callbacks are called to update upper layer analog/binary
					//records for first 8 banks only
					//Cant call GalilAxis->poll
					if (i < MAX_GALIL_AXES)
						pCntrl_->callParamCallbacks(i); 
					}
				else				
					pAxis->poll(&moving);		//Update GalilAxis, and upper layers, using retrieved datarecord
									//Update records with analog/binary data
				}
			//No async, so wait updatePeriod_ rather than relying on async record delivery frequency
			if (!pCntrl_->async_records_)
				epicsThreadSleep(pCntrl_->updatePeriod_/1000.0);
			}
		else	//Not connected.  Just sleep a little
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
	//Tell poller to shutdown
	shutdownPoller_ = true;
	//Pause
	epicsThreadSleep(.01);
	//Wake poller but do not restart async records
        wakePoller(false);
	//Wait till poller thread exits
	thread.exitWait();
}

GalilPoller::~GalilPoller()
{
	shutdownPoller();
}

//Put poller in sleep mode, and stop async records if needed
void GalilPoller::sleepPoller(bool connected)
{
	//Only if poller awake now
	if (!pollerSleep_)
		{
		//Tell poller to sleep
		pollerSleep_ = true;
		//Wait until GalilPoller is sleeping
		epicsEventWait(pollerSleepEventId_);
		//Tell controller to stop async record transmission
		if (pCntrl_->async_records_ && pCntrl_->gco_ != NULL && connected)
			{
			try 	{
				pCntrl_->gco_->recordsStart(0);
				}
			catch (string e)
				{
				//Print explanation
				cout << "Terminating async on poller sleep failed " << pCntrl_->model_ << " at " << pCntrl_->address_ << endl;
				cout << e << endl;
				}
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
		if (pCntrl_->async_records_ && pCntrl_->gco_ != NULL && restart_async)
			{
			try 	{
				pCntrl_->gco_->recordsStart(pCntrl_->updatePeriod_);
				}
			catch (string e)
				{
				//Print explanation
				cout << "Asynchronous re-start failed, swapping to synchronous poll " << pCntrl_->model_ << " at " << pCntrl_->address_ << endl;
				cout << e << endl;
				pCntrl_->async_records_ = false;
				}
			}
		}
}
