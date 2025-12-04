using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public abstract class GenericRobotController : MonoBehaviour
{
    public float linearSpeed = 4.0f;       // robot speed
    public float angularSpeed = 180f;     // degree/second
    public float reachThreshold = 0.5f;  // minimum distance to say that a target is reached
    public string robotId = string.Empty;

    protected enum MoveState { Rotating, Moving }
    protected MoveState moveState =  MoveState.Rotating;
    protected bool isMoving = false;

    protected List<Vector3> currentPath = new List<Vector3>(); //path from the current position of the robot to the target
    protected int currentPathIndex = 0;
    protected Vector3 currentTarget;
    
    public string topicNameTarget = "/target";
    public string topicNamePath = "/astar_path";

    protected bool waitingForPath;
    protected bool isReturningToBase = false;

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

        // Controllo se il punto è raggiunto
        if (distance <= reachThreshold)
        {
            Debug.Log($"Raggiunto punto [{currentPathIndex}] del path: {target}");

            currentPathIndex++;
            if (currentPathIndex >= currentPath.Count)
            {
                Debug.Log("Path completato!");
                isMoving = false;
                OnReachedTarget();
                return;
            }

            Debug.Log($"Prossimo punto [{currentPathIndex}] del path: {currentPath[currentPathIndex]}");
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
            // Conversione ROS2 -> Unity (Z verticale ignorata, Y=0 piano)
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

        Debug.Log($"<color=green>Path ricevuto da ROS. Lunghezza: {currentPath.Count}</color>");
    }

    protected void PublishTarget(Vector3 target)
    {
        PoseArrayMsg msg = new PoseArrayMsg();
        msg.poses = new PoseMsg[2];

        // Posizione robot
        Vector3 robotPos = transform.position;
        PoseMsg robotPose = new PoseMsg();
        robotPose.position = new PointMsg(robotPos.x, robotPos.y, robotPos.z);
        robotPose.orientation = new QuaternionMsg(0, 0, 0, 1);
        msg.poses[0] = robotPose;

        // Posizione target
        PoseMsg targetPose = new PoseMsg();
        targetPose.position = new PointMsg(target.x, target.y, target.z);
        targetPose.orientation = new QuaternionMsg(0, 0, 0, 1);
        msg.poses[1] = targetPose;

        ros.Publish(topicNameTarget, msg);

        Debug.Log($"<color=yellow>PoseArray pubblicato Robot: {robotPos} | Target: {target}</color>");
        waitingForPath = true;
    }
    protected abstract void OnReachedTarget();

}
