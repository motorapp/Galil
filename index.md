### Welcome

For assistance with this software post on EPICS tech-talk or email:  
cliftm@ansto.gov.au   
support@galil.com   

This driver has the following features:  

<li>Asyn model 3 architecture</li>
<li>Motor record support</li>
<li>Motor record PREM/POST function is supported</li>
<li>Analog and Digital IO</li>
<li>Output TTL pulses for closed-loop motor distance (eg. detector trigger)</li>
<li>Time based profile motion for real and coordinate system axes (Linear and PVT Trajectories)</li>
<li>Coordinate system axes (eg. Slit width, undulator taper, DCM energy)</li>
<li>Motor record backlash, and retry support for coordinate system axes</li>
<li>Accurate limit reporting for coordinate system axes</li>
<li>Kinematics adjustable via database puts</li>
<li>Deferred moves facility</li>
<li>True coordinated motion for up to 8 motors (Motors can be synchronized to within .2ms)</li>
<li>EtherCat motor drive support</li>
<li>BISS and SSI encoder support</li>
<li>Auto motor power On/Off with adjustable delays</li>
<li>Auto motor brake On/Off with adjustable delays</li>
<li>Sophisticated Galil code generator, no need to program low level</li>
<li>Ability to construct low level code from templates</li>
<li>Accepts custom code files</li>
<li>User definable control/monitor functions with full record support</li>
<li>Intr IO support for user defined records</li>
<li>Sophisticated connection management</li>
<li>Fast update rates 2ms</li>
<li>Example MEDM screens</li>
<li>Example Qt/EPICS QE framework screens</li>
<li>Example IOC</li>
<li>Autosave request files</li>
<li>RIO PLC supported</li>   
<br>
Intended to replace Galil EPICS 1-x driver   

Older versions can be found here:   

https://github.com/motorapp/Galil-3-0/releases   

I hope you find this software useful :)  
Mark Clift   
