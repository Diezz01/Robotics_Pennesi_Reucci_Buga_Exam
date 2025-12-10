using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public abstract class GenericRobotController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float linearSpeed = 4.0f;       // robot speed
    public float angularSpeed = 180f;     // degree/second
    public float reachThreshold = 0.5f;  // minimum distance to say that a target is reached
    public string robotId = string.Empty;

    [Header("Battery Management")]
    public BatterySimulator batterySimulator; // Direct reference to battery
    public Vector3 chargingStationPosition = Vector3.zero;
    public float chargedThreshold = 95f;       // Battery % to consider charging complete
    public float safetyCostMultiplier = 1.2f;  // Safety margin for cost estimates (20%)

    protected enum MoveState { Rotating, Moving }
    protected MoveState moveState =  MoveState.Rotating;
    protected bool isMoving = false;

    protected List<Vector3> currentPath = new List<Vector3>(); // path from the current position of the robot to the target
    protected int currentPathIndex = 0;
    protected MapTarget currentTarget;

    public string topicNameTarget = "/target";
    public string topicNamePath = "/astar_path";

    protected bool waitingForPath;
    protected bool isReturningToBase = false;
    protected bool isChargingMission = false;
    protected bool hasInsertedChargingMission = false;

    protected ROSConnection ros;

    // -----------------------------
    // ROBOT MOVEMENT FOLLOWING THE PATH  
    // -----------------------------
    protected void MoveAlongPathWithRotation()
    {
        if (currentPath.Count == 0 || !isMoving) return;

        Vector3 target = currentPath[currentPathIndex];
        Vector3 dir = new Vector3(
            target.x - transform.position.x,
            0f,
            target.z - transform.position.z
        );

        float distance = dir.magnitude;
        Vector3 dirNorm = dir.normalized;

        // Check if the point is reached
        if (distance <= reachThreshold)
        {
            Debug.Log($"Reached path point [{currentPathIndex}]: {target}");

            currentPathIndex++;
            if (currentPathIndex >= currentPath.Count)
            {
                Debug.Log("Path completed!");
                isMoving = false;
                OnReachedTarget();
                return;
            }

            Debug.Log($"Next path point [{currentPathIndex}]: {currentPath[currentPathIndex]}");
            moveState = MoveState.Rotating;
            return;
        }

        // calculating the angle to the target
        float angleToTarget = Vector3.SignedAngle(transform.forward, dirNorm, Vector3.up);

        // ------- ROTATION -------
        if (moveState == MoveState.Rotating)
        {
            if (Mathf.Abs(angleToTarget) > 2f)
            {
                float rotateStep = Mathf.Sign(angleToTarget) * angularSpeed * Time.deltaTime;
                rotateStep = Mathf.Clamp(rotateStep, -Mathf.Abs(angleToTarget), Mathf.Abs(angleToTarget));
                transform.Rotate(0f, rotateStep, 0f);
            }
            else
            {
                moveState = MoveState.Moving;
            }
            return;
        }

        // ------- MOVEMENT -------
        if (moveState == MoveState.Moving)
        {
            transform.position += transform.forward * linearSpeed * Time.deltaTime;
        }
    }

    protected void OnRosPathReceived(PathMsg msg)
    {
        waitingForPath = false;
        currentPath.Clear();

        foreach (var pose in msg.poses)
        {
            // ROS2 -> Unity conversion (Z vertical ignored, Y=0 plane)
            Vector3 p = new Vector3(
                (float)pose.pose.position.x,
                0f,
                (float)pose.pose.position.y
            );
            currentPath.Add(p);
        }

        currentPathIndex = 0;
        isMoving = true;
        moveState = MoveState.Rotating;

        Debug.Log($"<color=green>Path received from ROS. Length: {currentPath.Count}</color>");
    }

    protected void PublishTarget(Vector3 target)
    {
        PoseArrayMsg msg = new PoseArrayMsg();
        msg.poses = new PoseMsg[2];

        // Robot position
        Vector3 robotPos = transform.position;
        PoseMsg robotPose = new PoseMsg();
        robotPose.position = new PointMsg(robotPos.x, robotPos.y, robotPos.z);
        robotPose.orientation = new QuaternionMsg(0, 0, 0, 1);
        msg.poses[0] = robotPose;

        // Target position
        PoseMsg targetPose = new PoseMsg();
        targetPose.position = new PointMsg(target.x, target.y, target.z);
        targetPose.orientation = new QuaternionMsg(0, 0, 0, 1);
        msg.poses[1] = targetPose;

        ros.Publish(topicNameTarget, msg);

        Debug.Log($"<color=yellow>PoseArray published Robot: {robotPos} | Target: {target}</color>");
        waitingForPath = true;
    }
    protected abstract void OnReachedTarget();

}
