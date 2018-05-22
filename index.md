### Welcome

The EPICS driver advertised and supported by Galil should be considered a separate branch.  For assistance with this software post on EPICS tech-talk or email: Mark.Clift@Synchrotron.org.au

This driver has the following features:  

A) Asyn model 3 architecture  
B) Motor record support  
C) Motor record PREM/POST function is supported   
D) Analog and Digital IO  
E) Output TTL pulses for closed-loop motor distance (eg. detector trigger)   
F) Time based profile motion (Linear and PVT Trajectories)   
G) Coordinate system axes (eg. Slit width, undulator taper)   
H) Motor record backlash, retries supported across coordinate system axes   
I) Accurate limit support for coordinate system axes   
J) Kinematics adjustable via database puts   
K) Deferred moves facility   
L) True coordinated motion for up to 8 motors   
M) Auto motor power On/Off with adjustable delays  
N) Auto motor brake On/Off with adjustable delays   
O) Sophisticated Galil code generator, no need to program low level   
P) Ability to construct low level code from templates  
Q) Accepts custom code files   
R) User definable control/monitor functions with full record support
S) Intr IO support for user defined records   
T) Sophisticated connection management    
U) Fast update rates 2ms   
V) Example MEDM screens     
W) Example Qt/EPICS QE framework screens   
X) Example IOC  
Y) Autosave request files   
Z) RIO PLC supported   

Intended to replace Galil EPICS 1-5 driver  

Older versions can be found here:   

https://github.com/motorapp/Galil-3-0/releases   

I hope you find this software useful :)  
