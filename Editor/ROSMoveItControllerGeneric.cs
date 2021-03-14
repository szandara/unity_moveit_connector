using UnityEngine;
using ROSGeometry;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System;

/// <summary>
/// MoveIt controller for Unity and ROS. It handles the communication back and forth to ROS
/// for simulating a robotic arm. It sends the joint configuration of the arm at a fixed rate
/// emulating a real robot.
///
/// Furthermore it is able to accept moveit_msgs.RobotTrajectory messages and execute them
/// using the ArticulationBody object of Unity.
///
/// Author: Simone Zandara <simone.zandara@gmail.com>
/// </summary>
public class ROSMoveItControllerGeneric : MonoBehaviour
{
  // Variables required for ROS communication
  public string jointStateTopicName = "joint_states";
  public string moveItMoveGroup = "";

  // Manipulator arm root
  public GameObject jointGroup;

  // ROS Connector
  private ROSConnection ros;

  // Linux time calculation
  private readonly static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

  // ROS Topic to talk to Unity
  private readonly string moveItMoveGroupRoot = "/move_group/unity_trajectory/";

  // Maps the joint names with their position in the list of joints
	private Hashtable jointPositionIndex = new Hashtable();

  // Time waiting for xDrive to actuate the trajectory
  private readonly float jointAssignmentWait = 0.2f;

  // Publish rate of the joint
  private readonly float jointPublishRate = 0.1f; // 10Hz

  // Articulation Bodies ordered by their tree, this is the order used by /joint_states
  private List<ArticulationBody> jointArticulationBodies = new List<ArticulationBody>();

  /// <summary>
  /// Initialize the component by fetching all expected joints and starting to publish to ROS
  /// </summary>
  void Start()
  {
    // Get ROS connection static instance
    ros = ROSConnection.instance;

    // Get the root of the joint tree
    ArticulationBody root = jointGroup.GetComponent<ArticulationBody>();

    // Iteratively find all articulate bodies until we find the end effector
    int c = 0;
    while (root) {
      jointArticulationBodies.Add(root);
      var name = root.GetComponent<RosSharp.Urdf.UrdfJoint>().jointName;
      jointPositionIndex.Add(name, c);

      var next_joints = root.transform.GetComponentsInChildren<ArticulationBody>().
        Where(t => t.gameObject != root.gameObject);

      // No more joints
      if (next_joints.Count() == 0) {
        break;
      }

      root = next_joints.First();
      c += 1;
    }

    // Continuously publish the joint configuration of the robot
    InvokeRepeating("PublishJointStates", 1.0f, jointPublishRate);

    // Subscribe to the moveit trajectory topic
    ros.Subscribe<RosMessageTypes.Moveit.RobotTrajectory>
        (moveItMoveGroupRoot + moveItMoveGroup, TrajectoryHandler);
  }

  /// <summary>
  /// Set the proper timestamp for ROS
  /// </summary>
  private static RosMessageTypes.Std.Time now()
  {
    TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
    double msecs = timeSpan.TotalMilliseconds;
    uint secs = (uint)(msecs / 1000);
    uint nsecs = (uint)((msecs / 1000 - secs) * 1e+9);
    return new RosMessageTypes.Std.Time(secs, nsecs);
  }

  /// <summary>
  /// Publish the joint states from the current robot pose and
  /// send to ROS so that it can be used for motion planning
  /// </summary>
  public void PublishJointStates()
  {
    int numRobotJoints = jointArticulationBodies.Count;
    double[] jointPositions = new double[numRobotJoints];
    double[] jointVelocities = new double[numRobotJoints];
    double[] jointEfforts = new double[numRobotJoints];
    string[] jointNames = new string[numRobotJoints];

    jointPositionIndex.Keys.CopyTo(jointNames, 0);

    int c = 0;
    foreach (ArticulationBody body in jointArticulationBodies) {
      jointPositions[c] = body.jointPosition[0];
      jointVelocities[c] = body.jointVelocity[0];
      jointEfforts[c] = body.jointFriction;
      c += 1;
    }

    RosMessageTypes.Std.Header header = new RosMessageTypes.Std.Header();
    header.stamp = now();
    // Pick Pose
    RosMessageTypes.Sensor.JointState jointState = new RosMessageTypes.Sensor.JointState
    {
      header = header,
      name = jointNames,
      position = jointPositions,
      velocity = jointVelocities,
      effort = jointEfforts
    };

    // Finally send the message to server_endpoint.py running in ROS
    ros.Send(jointStateTopicName, jointState);
  }

  /// <summary>
  /// Start a separate routine which executes the trajectory
  /// </summary>
  public void TrajectoryHandler(RosMessageTypes.Moveit.RobotTrajectory response)
  {
    StartCoroutine(ExecuteTrajectories(response));
  }

  /// <summary>
  /// Execute the trajectory coming from MoveIt
  /// </summary>
  private IEnumerator ExecuteTrajectories(RosMessageTypes.Moveit.RobotTrajectory response)
  {
    // The joint order of MoveIt is not regular so we must read the names to map
    // the right joint index.
    string[] jointNamesFromROS = response.joint_trajectory.joint_names;

    // For each trajectory returned by MoveIt set the xDrive and wait for completion
    for (int jointConfigIndex  = 0 ; jointConfigIndex < response.joint_trajectory.points.Length; jointConfigIndex++)
    {
      // Get the joint position in radians and transform to degrees
      var jointPositions = response.joint_trajectory.points[jointConfigIndex].positions;
      float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

      // No points
      if (result.Length == 0)
      {
        Debug.Log("Joint trajectory returned 0 positions! Skipping");
        continue;
      }

      // Set the joint positions into the xDrive of the ArticulationBody
      for (int joint = 0; joint < result.Length - 1; joint++)
      {
        var jointName = jointNamesFromROS[joint];
        var jointIndex = (int)jointPositionIndex[jointName];
        var joint1XDrive = jointArticulationBodies[jointIndex].xDrive;
        joint1XDrive.target = result[joint];
        jointArticulationBodies[jointIndex].xDrive = joint1XDrive;
      }

      // Wait until the execution is complete
      yield return new WaitForSeconds(jointAssignmentWait);
    }
  }
}
