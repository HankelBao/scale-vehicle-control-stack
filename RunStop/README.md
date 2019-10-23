# RunStop

Provides a remote shutoff mechanism scale vehicle. The remote shutoff is triggered using a remote and receiver combination. Communication occurs over an WiFi station using the NodeMCU IoT module.

### RunStopRemote

A remote not directly attached to the vehicle. Has three buttons: activation, teleop, autonomous. When turning on, the remote must be activated first. Then teleop may be turned on, followed by autonomous mode. If all three have been successfully initialized, to initiate a remote shutdown, any button may be pressed. Teleop provides access to the user to control the vehicle through a handheld controller. Autonomous mode will listen to the onboard controller to perform actions specified over serial to the ArduinoInterface.

### RunStopRelay

Is actually on the vehicle. In the event of an emergency shutdown, a relay will be flipped into an off position, cutting power to the motors.

<img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/WA.png" alt="Wisconsin Autonomous Logo" height="100px">  <img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/UWCrest.png" alt="University of Wisconsin - Madison Crest" height="100px" align="right">
