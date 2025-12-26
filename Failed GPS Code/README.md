# Attempt at an autonomous quadcopter
In this folder you can find the latest code created with the sole purpose of building an autonomous drone.

The drone would receive missions through our custom UDP commands, the missions would contain a set of checkpoints.
Each containing a 3D coordinate that the drone would go through, as well as the flight mode at each checkpoint, for loittering, altitude hold or free flight among other modes.

## Features
- *Smart initialization*: When starting a function, the drone automatically stores the home position for return to launch (RTL) functionality.
- *Automatic takeoff function*: The drone takes off at a specified altitude and stays there until further command.
- *Safe landing*: The ground station pings the drone constantly, if the drone stops receiving ping packets from the ground station for more than 2 seconds it will automatically change flight mode to landing. Then it will safely and slowly descend until it reaches ground disarming itself on contact.
- *Emergency shutdown*: If the drone receives a special command, it will instantly stop the motors and disarm even while flying. This feature helps us prevent dangerous situations like the drone heading towards a person or an animal.
