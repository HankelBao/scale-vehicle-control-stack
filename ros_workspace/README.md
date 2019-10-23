# Scale Vehicle ROS Workspace

Full control stack for the scale vehicle.

#### 1. Perception

Perception is the first element of the control stack. The environment is analyzed.

#### 2. Localization and Estimation

The perceived environment is then reconstructed into a world. The vehicle and obstacles are then localized within this world.

#### 3. Control

Through the perceived environment and knowledge of the world and the vehicles position in it, the vehicle will now respond and attempt to travel through this world.

#### Drivers

This holds packages for reading data in from sensors and for specific data analysis not developed by Wisconsin Autonomous.

#### Common

This holds all of the common elements present in all other packages within the control stack. An example could be message, service and action types present in multiple nodes or common classes to be included/imported in many files.

<img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/WA.png" alt="Wisconsin Autonomous Logo" height="100px">  <img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/UWCrest.png" alt="University of Wisconsin - Madison Crest" height="100px" align="right">
