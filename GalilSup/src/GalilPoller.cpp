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
// We write our own because epicsEventWaitWithTimeout in asynMotorController::asynMotorPoller waits, we dont want that.
// Instead either Async speed determines poller frequency or simple epicsThreadSleep in Sync mode

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <epicsThread.h>
#include <shareLib.h>

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
   GalilAxis *pAxis; 	//Axis structure
   GalilCSAxis *pCSAxis;//CSAxis structure
   double time_taken;	//Time taken last poll cycle
   double sleep_time;	//Calculated time to sleep in synchronous mode

   //Loop until shutdown
   while (true) {
      //Dont poll in sleep mode or when shutting down
      if (!pollerSleep_ && !shutdownPoller_) {
         //Wake mode
         //Poll only if connected
         if (pC_->connected_) {
            //Read start time
            epicsTimeGetCurrent(&pollstartt_);

            //Get the data record, update controller related information in GalilController, and ParamList.  callBacks not called
            pC_->poller();

            //Update the GalilAxis status, using datarecord from GalilController
            //Do callbacks for GalilController, GalilAxis records
            //Do all ParamLists/axis whether user called GalilCreateAxis or not
            //because analog/binary IO data are stored/organized in ParamList just same as axis data 
            for (i=0; i<MAX_GALIL_AXES + MAX_GALIL_CSAXES; i++) {
               if (i < MAX_GALIL_AXES) {
                  //Retrieve GalilAxis instance i
                  pAxis = pC_->getAxis(i);
                  if (!pAxis) {
                     //User did not call GalilCreateAxis for this axis number
                     //Ensure callbacks are called to update upper layer analog/binary
                     //Do this for first 8 addresses only to cover analog/binary
                     //Cant call GalilAxis->poller
                     pC_->callParamCallbacks(i);
                  }
                  else {
                     //Update GalilAxis, and upper layers, using retrieved datarecord
                     //Update records with analog/binary data
                     pAxis->poller();
                  }
               }
               else {
                  //Retrieve GalilCSAxis instance i
                  pCSAxis = pC_->getCSAxis(i);
                  if (pCSAxis) {
                     //Update GalilCSAxis, and upper layers, using retrieved datarecord
                     //Update records with analog/binary data
                     pCSAxis->poller();
                  }
               }
            } //For

            //Read end time
            epicsTimeGetCurrent(&pollendt_);
            //Calculate cycle time
            time_taken = epicsTimeDiffInSeconds(&pollendt_, &pollstartt_);

            //No async, so wait updatePeriod_ rather than relying on async record delivery frequency
            if (!pC_->async_records_) {
               //Adjust sleep time according to time_taken last poll cycle
               sleep_time = pC_->updatePeriod_/1000.0 - time_taken;
               //Must sleep in synchronous mode to release lock for other threads
               sleep_time = (sleep_time < 0.000) ? 0.001 : sleep_time;
               if (sleep_time >= 0.001)
                  epicsThreadSleep(sleep_time);
            }
         } //Connected_
         else //Not connected so sleep a little
            epicsThreadSleep(.1);
      }
      else if (pollerSleep_ && !shutdownPoller_) {
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
   //Shutdown poller thread
   shutdownPoller();
   //Destroy events
   epicsEventDestroy(pollerSleepEventId_);
   epicsEventDestroy(pollerWakeEventId_);
}

//Put poller in sleep mode, and stop async records if needed
//Put poller in sleep mode, and stop async records if needed
//Must be called without lock so sync poller can be put to sleep
void GalilPoller::sleepPoller(bool waitTillSleep)
{
   //Only if poller awake now
   if (!pollerSleep_) {
      //Tell poller to sleep
      pollerSleep_ = true;   
      //Wait until GalilPoller is sleeping, if requested by caller
      if (true == waitTillSleep) {
         epicsEventWait(pollerSleepEventId_);
      }
      //Tell controller to stop async record transmission
      if (pC_->async_records_ && pC_->connected_) {
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
   int status;

   //Only if poller sleeping now
   if (pollerSleep_) {
      //Wake up poller
      pollerSleep_ = false;
      epicsEventSignal(pollerWakeEventId_);
      if (pC_->connected_) {
         //Turn on data record transmission if requested
         if (pC_->try_async_ && restart_async) {
            //Operator requests async UDP
            //Turn on data record transmission
            sprintf(pC_->cmd_, "DR %.0f, %d", pC_->updatePeriod_, pC_->udpHandle_ - AASCII);
            status = pC_->sync_writeReadController();
            if (status) {
               pC_->async_records_ = false; //Something went wrong
               pC_->setCtrlError("Asynchronous UDP failed, switching to TCP synchronous");
            }
            else {
               pC_->async_records_ = true; //All ok
            }
         }
         //Set connection that will receive unsolicited messages
         if (pC_->async_records_)
            sprintf(pC_->cmd_, "CF %c", pC_->udpHandle_);
         else
            sprintf(pC_->cmd_, "CF %c", pC_->syncHandle_);  
         status = pC_->sync_writeReadController();
         //Set most signficant bit for unsolicited bytes
         strcpy(pC_->cmd_, "CW 1");
         status = pC_->sync_writeReadController();
      }
   }
}
