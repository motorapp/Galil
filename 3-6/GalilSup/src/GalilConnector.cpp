// Copyright (c) 2014  Australian Synchrotron
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
// cliftm@ansto.gov.au
// 800 Blackburn Road, Clayton, Victoria 3168, Australia.
//
// Thread to verify device actually responds when asyn connected true
// We maintain our own connected_ flag outside of asyn

#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <algorithm> //std::count

using namespace std; //cout ostringstream vector string

#include "GalilController.h"

//Constructor
GalilConnector::GalilConnector(GalilController *pcntrl)
   :  thread(*this,"GalilConnector",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityLow)
{
	//Store the GalilController instance that created this GalilConnector instance
	pC_ = pcntrl;
	//Flag not connected at startup
	pC_->connected_ = false;
	//Flag GalilConnector thread not shutting down
	shutDownConnector_ = false;
	//Start GalilConnector thread
	thread.start();
}

//Destructor
GalilConnector::~GalilConnector()
{
	//Flag to GalilConnector run thread that IOC is shutting down
	shutDownConnector_ = true;
	//Wake GalilConnector thread now shutdown flag is set
	epicsEventSignal(pC_->connectEvent_);
	//Wait for run thread to exit
	thread.exitWait();
}

//GalilConnector thread
//Thread to verify device actually responds when asyn connected true
//This is needed due to Moxa use
//We maintain our own connected_ flag outside of asyn
void GalilConnector::run(void)
{
   int sync_status;		//Synchronous communication status
   int async_status = asynSuccess; //Asynchronous communication status
   string resp;			//For checking controller response

   //Check if Galil actually responds to query
   while (true) {
      //Wait for connect event signal
      epicsEventWait(pC_->connectEvent_);
      if (shutDownConnector_) {
         //Thread shutdown requested
         break;
      }
      else {
         pC_->lock();
         //Check GalilController for response
         //Test synchronous communication
         //Query controller for synchronous connection handle
         strcpy(pC_->cmd_, "WH");
         sync_status = pC_->sync_writeReadController(true);
         //Store the handle controller used for sync
         if (!sync_status) {
            if (strncmp(pC_->resp_, "IH", 2) == 0)
               pC_->syncHandle_ = pC_->resp_[2];//TCP
            else if (strncmp(pC_->resp_, "RS", 2) == 0)
               pC_->syncHandle_ = 'S';//Serial
            else//Bad response from controller
               sync_status = asynError;
            //Check response to QZ command
            if (!sync_status) {
               strcpy(pC_->cmd_, "QZ");
               sync_status = pC_->sync_writeReadController(true);
               //Check response from controller
               if (!sync_status) {
                  //Copy response to std::string for convenience
                  resp = pC_->resp_;
                  //QZ response should contain 3 comma's
                  if (count(resp.begin(), resp.end(), ',') != 3)
                     sync_status = asynError; //Bad response
               }
            }
         }
         //Check asynchronous communication
         if (pC_->try_async_ && !sync_status) {
            //Ensure data record transmission is off
            strcpy(pC_->cmd_, "DR 0");
            sync_status = pC_->sync_writeReadController(true);
            //Close all other connections on controller
            strcpy(pC_->cmd_, "IHT=>-3");
            sync_status = pC_->sync_writeReadController(true);
            //Open UDP connection, and retrieve connection handle
            strcpy(pC_->asynccmd_, "WH\r");
            async_status = pC_->async_writeReadController();
            async_status = (strncmp(pC_->asyncresp_, "IH", 2) == 0) ? async_status : asynError;
            //Store the handle controller used for udp
            if (!async_status) {
               //Udp connection is responsive to query
               pC_->udpHandle_ = pC_->asyncresp_[2];
               pC_->async_records_ = true;
            }
            else//Error when querying udp handle
               pC_->async_records_ = false;
         }
         //Work out what to do
         if (!sync_status && !async_status)	//Response received
            pC_->connected();//Do whats required for GalilController once connection established
         else if (!pC_->shuttingDown_) {
            //IOC isn't shutting down
            //No response
            //Continue to force disconnect until device responds
            pC_->disconnect();
         }
      pC_->unlock();
      }
   } //while true
}

