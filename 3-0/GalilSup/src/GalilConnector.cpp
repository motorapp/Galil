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
// mark.clift@synchrotron.org.au
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
	shuttingDown_ = false;
	//Start GalilConnector thread
	thread.start();
}

//Destructor
GalilConnector::~GalilConnector()
{
	//Flag to GalilConnector run thread that IOC is shutting down
	shuttingDown_ = true;
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
        int sync_status;
        int async_status = asynSuccess;
        char term[] ={(char)0x8d,(char)0x8a};	//Terminator for unsolicted messages

	//Check if Galil actually responds to query
	while ( true )
		{
		//Wait for connect event signal
		epicsEventWait(pC_->connectEvent_);
		if (shuttingDown_)
			break; // exit outer while loop
		else
			{
                        pC_->lock();
			//Check GalilController for response
                        //Test synchronous communication
                        //Query controller for synchronous connection handle
			strcpy(pC_->cmd_, "WH");
			sync_status = pC_->sync_writeReadController();
                        if (!sync_status)
                          pC_->syncHandle_ = pC_->resp_[2];	//Store the handle controller used for sync
                        //Check asynchronous communication
                        if (pC_->try_async_)
                           {
                           //Need to change terminator for handle discovery query on udp connection
                           pasynOctetSyncIO->setInputEos(pC_->pasynUserAsyncGalil_, "\r\n", 2);
                           //Retrieve controller connection handle used for async udp
                           strcpy(pC_->asynccmd_, "WH");
                           async_status = pC_->async_writeReadController();
                           //Change terminator back to that required for receiving unsolicted messages
                           pasynOctetSyncIO->setInputEos(pC_->pasynUserAsyncGalil_, term, 2);

                           if (!async_status)
                              {
                              pC_->udpHandle_ = pC_->asyncresp_[2];	//Store the handle controller used for udp
                              pC_->async_records_ = true;	//Udp connection is responsive to query
                              }
                           else
                              pC_->async_records_ = false;	//Error when querying udp handle
                           }
                        //Work out what to do
			if (!sync_status && !async_status)	//Response received
				pC_->connected();//Do whats required for GalilController once connection established
			else
				{
				//No response
                                //Keep sync and asyn flags in same state
				pC_->connected_ = false;
				pC_->setIntegerParam(0, pC_->GalilCommunicationError_, 1);
				//Continue to force disconnect until device responds
				pC_->disconnect();
				}
                        pC_->unlock();
			}
		}
}

