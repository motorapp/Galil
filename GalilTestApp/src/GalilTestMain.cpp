/* GalilTestMain.cpp */
/* Author:  Marty Kraimer Date:    17MAR2000 */

#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "epicsThread.h"
#include "iocsh.h"
#include "epicsExit.h"

int main(int argc,char *argv[])
{
    if(argc>=2) {    
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    iocsh(NULL);
    
    epicsExit(EXIT_SUCCESS);  // ensures epics exit handlers are called
    // Note that the following statement will never be executed
    return(0);
}
