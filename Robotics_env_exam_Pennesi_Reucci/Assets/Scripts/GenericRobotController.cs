using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
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
    public float reachThreshold = 1.0f;  // minimum distance to consider target reached (increased for smoother paths)
    public string robotId = string.Empty;
    public int robotIndex = 0; // Set by MapGenerator during spawn

    [Header("Battery Management")]
    public BatterySimulator batterySimulator; // Direct reference to battery
    public Vector3 chargingStationPosition = Vector3.zero;
    public float chargedThreshold = 95f;       // Battery % to consider charging complete
    public float safetyCostMultiplier = 1.2f;  // Safety margin for cost estimates (20%)

    [Header("Debugging")]
    public bool enableMovementDebugLogs = false;

    protected enum MoveState { Rotating, Moving }
    protected MoveState moveState =  MoveState.Rotating;
    protected bool isMoving = false;

    protected List<Vector3> currentPath = new List<Vector3>(); // path from the current position of the robot to the target
    protected int currentPathIndex = 0;
    protected MapTarget currentTarget;

    public string topicNameTarget = "/target";
    public string topicNamePath = "/astar_path";
    protected string topicNamePose = "/pose";
    protected string topicNameCollision = "/collision_detected";

    protected bool waitingForPath;
    protected bool collisionDetected = false;  // Emergency stop flag
    private float posePublishRate = 0.1f;      // Publish position at 10Hz
    private float posePublishTimer = 0f;
    protected bool isReturningToBase = false;
    protected bool isChargingMission = false;
    protected bool hasInsertedChargingMission = false;

    // Collision waiting timeout
    private float collisionStopTime = 0f;
    private float maxWaitTime = 8.0f;  // Maximum seconds to wait before rerouting (synced with ROS)

    // Intelligent replanning
    private int rerouteAttempts = 0;
    private float lastRerouteTime = 0f;

    protected ROSConnection ros;
    protected bool isFullyInitialized = false;  // Track complete initialization to prevent publishing before setup

    // -----------------------------
    // POSITION PUBLISHING & COLLISION DETECTION
    // -----------------------------
    protected void UpdatePositionPublishing()
    {
        posePublishTimer += Time.deltaTime;

        if (posePublishTimer >= posePublishRate)
        {
            PublishPosition();
            posePublishTimer = 0f;
        }
    }

    protected void PublishPosition()
    {
        // Don't publish if ROS connection not fully initialized yet
        if (!isFullyInitialized) return;

        PoseStampedMsg poseMsg = new PoseStampedMsg();
        poseMsg.header.stamp.sec = (int)Time.time;
        poseMsg.header.stamp.nanosec = (uint)((Time.time - (int)Time.time) * 1e9);
        poseMsg.header.frame_id = "map";

        Vector3 pos = transform.position;
        poseMsg.pose.position = new PointMsg(pos.x, pos.y, pos.z);
        poseMsg.pose.orientation = new QuaternionMsg(0, 0, 0, 1);

        ros.Publish(topicNamePose, poseMsg);
    }

    protected void OnCollisionDetected(BoolMsg msg)
    {
        bool previousState = collisionDetected;
        collisionDetected = msg.data;

        if (collisionDetected && !previousState)
        {
            Debug.Log($"<color=red>{robotId}: COLLISION DETECTED - STOPPING</color>");
        }
        else if (!collisionDetected && previousState)
        {
            Debug.Log($"<color=lime>{robotId}: Path clear - RESUMING</color>");
        }
    }

    // -----------------------------
    // ROBOT MOVEMENT FOLLOWING THE PATH
    // -----------------------------
    protected void MoveAlongPathWithRotation()
    {
        if (currentPath.Count == 0 || !isMoving) return;

        // EMERGENCY STOP: If collision detected, pause movement
        if (collisionDetected)
        {
            // Track how long we've been stopped
            collisionStopTime += Time.deltaTime;

            if (collisionStopTime > maxWaitTime)
            {
                Debug.LogWarning($"{robotId}: Waited {collisionStopTime:F1}s - requesting alternate path");

                // Exponential backoff for reroute attempts
                rerouteAttempts++;
                float backoffDelay = Mathf.Min(rerouteAttempts * 2f, 10f);

                if (Time.time - lastRerouteTime < backoffDelay)
                {
                    return;  // Wait for backoff period
                }

                lastRerouteTime = Time.time;
                collisionStopTime = 0f;

                // Try lateral offset path (first 3 attempts)
                if (rerouteAttempts <= 3 && currentTarget != null)
                {
                    Vector3 lateralOffset = Vector3.Cross(Vector3.up, transform.forward).normalized * 5f;
                    Vector3 offsetGoal = currentTarget.Position + lateralOffset;

                    Debug.Log($"{robotId}: Attempt {rerouteAttempts} - trying offset path to {offsetGoal}");
                    PublishTarget(offsetGoal);
                }
                else if (currentTarget != null)
                {
                    // After 3 attempts, try original goal again
                    Debug.Log($"{robotId}: Reattempting original goal");
                    PublishTarget(currentTarget.Position);
                    rerouteAttempts = 0;  // Reset
                }

                // DON'T clear collisionDetected here - let ROS handle it
            }
            return;  // Don't move until collision clears
        }
        else
        {
            // Collision cleared - reset timer
            collisionStopTime = 0f;
        }

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
            if (enableMovementDebugLogs)
            {
                Debug.Log($"Reached path point [{currentPathIndex}]: {target}");
            }

            currentPathIndex++;
            if (currentPathIndex >= currentPath.Count)
            {
                Debug.Log("Path completed!");
                isMoving = false;
                OnReachedTarget();
                return;
            }
            if (enableMovementDebugLogs)
            {
                Debug.Log($"Next path point [{currentPathIndex}]: {currentPath[currentPathIndex]}");
            }
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
        rerouteAttempts = 0;  // Reset on successful path

        Debug.Log($"<color=green>Path received from ROS. Length: {currentPath.Count}</color>");
    }

    protected void PublishTarget(Vector3 target)
    {
        // Don't publish if ROS connection not fully initialized yet
        if (!isFullyInitialized)
        {
            Debug.LogWarning($"{robotId}: Cannot publish target - ROS not fully initialized yet");
            return;
        }

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
