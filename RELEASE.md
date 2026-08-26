# Galil: Release Notes

## Release 4-1-16 (26/08/2026)
- GalilAxis::setDefaults
  - Added defaults for GalilError_, GalilMotorVelocityRAW_, GalilMotorConnected_ asyn parameters to avoid value not defined exceptions

- GalilController::setCtrlError
  - Changed to pass by reference

- GalilController::sourceValue
  - Rewritten for heap allocation and performance optimization

- GalilController::sync_writeReadController
  - Revised for heap allocation and performance optimization

- GalilAxis::setAccelVelocity
  - Revised for heap allocation and performance optimization

- GalilAxis::axisStatusThread()
  - Causing paramVal::getInteger value not defined errors and excess mallocs
  - Parameters GalilSSICapable_ and GalilBISSCapable_ are set for list 0 only
   GalilAxis::axisStatusThread() adjusted so it obtains the values only from list 0

- Update display shell scripts to be compatible with the SUPPORT macro
- Set parameters GalilUserArrayUpload_, GalilEtherCatCapable_ to default 0 at startup
  - Was causing paramVal::getInteger value not defined errors and excess mallocs
