Galil-3-3
=========

ASYN based EPICS driver for Galil products

Notes
=====

If using RS232 communication on Microsoft Windows, need XON/XOFF flow control enabled via switches on Galil controller or else uploading
a program to the controller times out and fails. However, all other read/write communication works fine without flow control enabled.
