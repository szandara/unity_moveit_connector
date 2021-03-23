## Intro
Unity script which acts as controller for Unity ArticulationBody robotics arms for a ROS moveit instance. This C# script can be used to use Unity as simulator for ROS. It follows the examples from https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/3_pick_and_place.md but tries to generify the creation of the state publishing and joint actuation.

The script does two things:
- It streams the joint state of the arm to ROS
- It receives ROS trajectories and actuates the joints of an articulated arm using ArticulationBody

It requires Unity version >= 2020.05f

A convenient way to use this component is joint with https://github.com/szandara/unity_moveit_manager to stream the trajectories from MoveIt! direclty to Unity.
The C# script should be added to the Unity scene and configured to point to the base of an articulated arm. Only articulated arms are supported.

## Installation in Unity
First install in Unity the following packages using the Package Manager
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git#v0.1.2 
https://github.com/Unity-Technologies/URDF-Importer.git#v0.1.2
```
Then paste the content of the [C# script](https://github.com/szandara/unity_moveit_connector/blob/master/Editor/ROSMoveItControllerGeneric.cs) into your assets and make sure it compiles.

## Example with Reachy
Here we setup a script to control Unity's ArticulationBody objects with MoveIt! messages from ROS. I have uploaded the script in this repository https://github.com/szandara/unity_moveit_connector/blob/master/Editor/ROSMoveItControllerGeneric.cs

* Copy paste the content of [the file](https://github.com/szandara/unity_moveit_connector/blob/master/Editor/ROSMoveItControllerGeneric.cs) directly into a file named *Assets/moveit/ROSMoveItControllerGeneric.cs* make sure Unity compiles it without errors.
* Finally, add the script to Reachy by selecting *r_shoulder* then *Add Component* -> *Scripts* -> *ROS Move It Controller Generic*

* Add the parameters as seen in this example
  * *Joint State Topic Name*: joint_states
  * *Move It Move Group*: unity_right_arm_controller
  * *Joint Group*: Drag and Drop the *r_shoulder* joint from the Unity tree as visible in this screenshot.

![image](https://dev-to-uploads.s3.amazonaws.com/uploads/articles/248scen0vlquo9vovn5h.png)

See full tutorial: https://dev.to/szandara/robotics-on-wsl2-using-ros-docker-and-unity-3d-part-i-3752
