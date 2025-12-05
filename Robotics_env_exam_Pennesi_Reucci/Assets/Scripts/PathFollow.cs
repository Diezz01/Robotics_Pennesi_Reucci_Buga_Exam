using RosMessageTypes.Nav;
using System.Collections.Generic;
// Import ROS�TCP Connector
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEngine;
using static UnityEditor.PlayerSettings;

public class PathFollower : MonoBehaviour
{
    [Header("Movement")]
    public float linearSpeed = 1.0f;         // m/s
    public float angularSpeed = 1.0f;        // rad/s
    public float reachThreshold = 0.05f;     // minimum distance to consider waypoint reached
    public float wheelBase = 0.16f;          // distance between wheels (m)
    public float wheelRadius = 0.033f;
    private enum MoveState { Rotating, Moving };
    private MoveState state = MoveState.Rotating;

    [Header("Wheel Joints (ArticulationBody)")]
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    [Header("ROS")]
    public string topicName = "/astar_path";

    private List<Vector3> path = new List<Vector3>();
    private int currentIndex = 0;
    private bool isMoving = false;
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathMsg>(topicName, OnRosPathReceived);
    }
    void FixedUpdate()
    {
        if (!isMoving || path.Count == 0) return;

        
        Vector3 target = path[currentIndex];
       
        Vector3 robotForward = (rightWheel.transform.position - leftWheel.transform.position).normalized; ;
        robotForward.y = 0f;
       // float distance = robotForward.magnitude;
        

        Vector3 dirXZ = new Vector3(target.x - transform.position.x, 0f, target.z - transform.position.z);
        float distance = dirXZ.magnitude;
        dirXZ.Normalize();

        float angleToTarget = Vector3.SignedAngle(robotForward, dirXZ, Vector3.up) * Mathf.Deg2Rad;

        // STEP 1 If we're close to the point, move to the next one
        if (distance <= reachThreshold)
        {
            currentIndex++;
            if (currentIndex >= path.Count)
            {
                StopWheels();
                isMoving = false;
                Debug.Log("Path completed!");
                return;
            }

            // new point = restart with rotation
            state = MoveState.Rotating;
            return;
        }

        // STEP 2 - Rotate until aligned
        float angleDegrees = Mathf.Abs(angleToTarget * Mathf.Rad2Deg);//TODO UNDERSTAND WHY ANGLE DEGREES DO NOT CHANGE EVEN IF THE ROBOT ROTEATE

        if (state == MoveState.Rotating)
        {
            Debug.Log("Angle Degrees: "+ angleDegrees);
            if (angleDegrees > 2f)
            {
                Debug.Log("STATE: Rotating. Angle to target: "+ angleToTarget+ "Angular speed: "+ angularSpeed);
                
                // Rotation in place
                float w = Mathf.Clamp(angleToTarget * 2f, -angularSpeed, angularSpeed);

                //float leftSpeed = -w * (wheelBase / 2f);
                //float rightSpeed = w * (wheelBase / 2f);
                float leftSpeed = -w * (wheelBase * 6);
                float rightSpeed = w * (wheelBase * 6);
                float minWheelSpeed = 0.05f; // m/s
                leftSpeed = Mathf.Sign(leftSpeed) * Mathf.Max(Mathf.Abs(leftSpeed), minWheelSpeed);
                rightSpeed = Mathf.Sign(rightSpeed) * Mathf.Max(Mathf.Abs(rightSpeed), minWheelSpeed);

                Debug.Log("Wheels speed. : leftSpeed: " + leftSpeed + "rightSpeed: " + rightSpeed);
                SetWheelTarget(leftWheel, leftSpeed);
                SetWheelTarget(rightWheel, rightSpeed);
            }
            else
            {
                Debug.Log("STATE: Moving");
                // Aligned, start moving
                state = MoveState.Moving;
            }
            return;
        }
        
        



        // STEP 3 Movement towards target
        float v = linearSpeed;
        float wMove = Mathf.Clamp(angleToTarget * 2f, -angularSpeed, angularSpeed);

        float left = v - wMove * (wheelBase / 2f);
        float right = v + wMove * (wheelBase / 2f);

        SetWheelTarget(leftWheel, left);
        SetWheelTarget(rightWheel, right);
    }

    void SetWheelTarget(ArticulationBody wheel, float speed)
    {
        // speed = linear velocity of the wheel in m/s
        float wheelRadPerSec = speed / wheelRadius; // convert to rad/s
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = wheelRadPerSec * Mathf.Rad2Deg; // targetVelocity in degrees/s
        wheel.xDrive = drive;
    }


    void StopWheels()
    {
        SetWheelTarget(leftWheel, 0f);
        SetWheelTarget(rightWheel, 0f);
    }

    public void SetPath(List<Vector3> newPath)
    {
        path = newPath;
        currentIndex = 0;
        isMoving = true;

        Debug.Log($"Received new path with {newPath.Count} points");
    }

    private void OnRosPathReceived(PathMsg msg)
    {
        List<Vector3> newPath = new List<Vector3>();

        foreach (var poseStamped in msg.poses)
        {
            // ROS -> Unity coordinate conversion
            newPath.Add(new Vector3(
                (float)poseStamped.pose.position.x,
                0f,
                (float)poseStamped.pose.position.y
            ));
        }

        SetPath(newPath);
    }
}
