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
// Single thread to manage disconnect/connect for all registered GalilController(s)

#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <Galil.h>
#include <epicsThread.h>

using namespace std; //cout ostringstream vector string

#include "GalilController.h"

GalilConnector::GalilConnector(void)
   :  thread(*this,"GalilConnector",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityLow)
{
	//Start GalilConnector thread
	shuttingDown_ = false;
	thread.start();
}

void GalilConnector::shutdownConnector()
{
	shuttingDown_ = true;
	thread.exitWait();
}

GalilConnector::~GalilConnector()
{
	shutdownConnector();
}

//GalilConnector thread
//Ask registered GalilControllers to check need for disconnect/connect
void GalilConnector::run(void)
{
	unsigned i;
	//Check to see what controllers need disconnect/connect
	while ( true )
		{
		//Loop through stored GalilControllers
		if (shuttingDown_)
			{
			for (i=0;i<pCntrlList_.size();i++)
				{
				pCntrlList_[i]->shutdownController();
				pCntrlList_[i]->disconnect();
				delete pCntrlList_[i];
				}
			pCntrlList_.clear();
			break; // exit outer while loop
			}
		else
			{
			for (i=0;i<pCntrlList_.size();i++)
				{
				//Ask GalilController instance to check need for disconnect/connect
				//And do it if needed.
				pCntrlList_[i]->connectManager();
				}
			}
		//1Hz Cycle for GalilConnector
		epicsThreadSleep(1);
		}
}

//Register a controller for connection management
void GalilConnector::registerController(GalilController *pCntrl)
{
	//Store GalilController instance
	pCntrlList_.push_back(pCntrl);
}


