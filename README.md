# Braccio Pick+Drop Simulation with ROS MoveIt and Gazebo
This is a package forked from lots-of-things repository, updated to Ros noetic and python3.

This is a standalone simulation of a Braccio arm with a block and a ramp.  The robot arm picks the red block both from above or from the side, depending on position, and delivers the 
block to the ramp. 

The package comes with a command line interface to control the simulated arm.

## Installation

### Prequisities
*  Tested on ROS NOETIC
*  Required Packages:
```
sudo apt-get install ros-noetic-gazebo-ros-pkgs 
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-moveit
```

### Download and build source
```
mkdir braccio_arm_ws
cd braccio_arm_ws
mkdir src
cd src
git clone git@github.com:lots-of-things/braccio_moveit_gazebo.git
cd ..
catkin_make
```

## Usage

### Launch the programs

Start up two bash terminal sessions and navigate to the workspace directory (`braccio_arm_ws` above).

In the first terminal run the following to bring up the Gazebo simulation.
```
source devel/setup.bash
roslaunch braccio_moveit_gazebo rviz_connected_with_gz_using_moveit.launch
```

Gazebo should open with a scene that contains the robot and the items.  You may need to reposition the display camera to see everything.

![Gazebo scene](doc/gazebo_open.png)

In the second terminal run this to bring up the command line control interface
```
source devel/setup.bash
rosrun braccio_moveit_gazebo target_object_sim.py
```

You should see the following splash screen after a few seconds
![Command Line Program Welcome](doc/cmd_line_welcome.png)


### Using the control interface
After you've launched the program a menu will pop up with options to

Hopefully the commands are pretty self-explanatory.
```
==================== Instructions: ====================
c = calibrate, rerun calibration routine
t = target, pick up red block and drop on the ramp
m = manual, manually enter location for pickup
b = bowl, go through the preprogrammed bowl move
r = reset_target, set block to new location, reset bowl
e = evaluate, test pickup and collect statistics to file
q = quit program
```

Importantly, if you press `t` or `m` you'll be further prompted for whether you want to try to pickup from above or from the side.  The script will make a best attempt to find a suitable combination of joint positions to perform a pickup.  If that isn't possible, it'll attempt to push the object into position. If that isn't possible it'll just abort with a prompt.

To reset, press `r` and then insert the (x, y) position for the red block to be reset to.  The robot can reach within a radius of about 0.5 from the origin (0,0).

Finally, to run the evaluation script press `e` and enter the number of trials.  It takes probably 20-30 seconds per trial.  The results willbe saved to `eval_results.json` for later use.

## Learn more

I've written up a more elaborate [blog post](#), that explains the internals in more detail. It covers the inverse kinematics solution, Gazebo+Moveit URDF setup, and Gazebo physics engine fine-tuning.

This project is a subtask in the creation of my [`su_chef` project](https://bonkerfield.org/su_chef/), an open-source, automated food prep assistant.

## Contact

For bugs, you can file [an issue](https://github.com/caesar753/braccio_moveit_gazebo/issues) and I'll take a look.

## Credits
