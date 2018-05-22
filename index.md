### Welcome

The EPICS driver advertised and supported by Galil should be considered a separate branch.  For assistance with this software post on EPICS tech-talk or email: Mark.Clift@Synchrotron.org.au

This driver has the following features:  

A) Asyn model 3 architecture  
B) Motor record support  
C) Motor record PREM/POST function is supported   
D) Analog and Digital IO  
E) Output TTL pulses for closed-loop motor distance (eg. detector trigger)   
F) Time based profile motion (Linear and PVT Trajectories)   
G) Coordinate system axes (eg. Slit width)   
H) Motor record backlash, retries supported across coordinate systems   
I) Kinematics adjustable via database puts   
J) Deferred moves facility   
K) True coordinated motion for up to 8 motors   
L) Auto motor power On/Off with adjustable delays  
M) Auto motor brake On/Off with adjustable delays   
N) Sophisticated Galil code generator, no need to program low level   
O) Ability to construct low level code from templates  
P) Accepts custom code files   
Q) User definable control/monitor functions with full record support
R) Intr IO support for user defined records   
S) Sophisticated connection management    
T) Fast update rates 2ms   
U) Example MEDM screens     
V) Example Qt/EPICS QE framework screens   
W) Example IOC  
X) Autosave request files   
Y) RIO PLC supported   
Z) Intended to replace Galil EPICS 1-5 driver  

Older versions can be found here:   

https://github.com/motorapp/Galil-3-0/releases   

I hope you find this software useful :)  
