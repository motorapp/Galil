#include <epicsExport.h>

//Dummy iocsh function register
static void GalilSupportRegister(void)
{
}

//Finally do the registration
extern "C" {
epicsExportRegistrar(GalilSupportRegister);
}
