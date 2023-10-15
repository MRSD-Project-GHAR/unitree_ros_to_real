# Notes for our Reference. 

## Robot Startup 
1. Make sure battery is charged, 
	- there is no recovery behaviour if the battery is over, the robot will fall.

2. short press and long press to turn on the robot, same for the joystick as well. 
3. Robot will stand up if the boot is successful. 

4. `L2+a` twice to sit
5. `L2+b` to completely lie down on ground
    - Do this stepwise as 

## Robot Compute
In total there are 4 computers. 
- NX
- Pi
- 2 Jetson Nano

## Network Config

### Connecting through the local Unitree network
wifi name: `Unitree_Go271911A`   
password: `00000000`

connect to pi (password: 123): 
```
ssh pi@192.168.12.1
```

connect to unitree (password: 123): 
```
ssh unitree@192.168.123.15
```
### Connecting through the CMU Device 

connect to unitree (password: 123): 
```
ssh unitree@172.26.172.181
```

## Control the robot

### Through Keyboard

Make sure the robot is standing after the bootup as always. 
```
roslaunch unitree_legged_real keyboard_control.launch
```
If this doesn't work, you need to try restarting the robot. 

### Through Keyboard with Joystick E-stop

Make sure the robot is standing after the bootup as always. 
```
roslaunch unitree_legged_real keyboard_control_with_estop.launch
```
If this doesn't work, you need to try restarting the robot. 

In our case we are using the `A` button as e-stop.    
Note: Even though we force the high level velocities to be zero, the robot still takes some action to stabilize itself. 

### Through rostopic pub
Before publishing these topics you need to make sure the `twist_sub` node is running. 
```
roslaunch unitree_legged_real twist_sub.launch
```

When running the below command it will continously take 0.5 as x vel. You need to make sure you give 0.0 again in sometime. 
```
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: \
  x: 0.5 \
  y: 0.0 \
  z: 0.0 \
angular: \
  x: 0.0 \
  y: 0.0 \
  z: 0.0" 
```

```
rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "linear: \
  x: 0.0 \
  y: 0.0 \
  z: 0.0 \
angular: \
  x: 0.0 \
  y: 0.0 \
  z: 0.0" 
```
