using RosMessageTypes.Nav;
using System.Collections.Generic;
// Import ROS–TCP Connector
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEngine;
using static UnityEditor.PlayerSettings;

public class PathFollower : MonoBehaviour
{
    [Header("Movement")]
    public float linearSpeed = 1.0f;         // m/s
    public float angularSpeed = 1.0f;        // rad/s
    public float reachThreshold = 0.05f;     // distanza minima per considerare waypoint raggiunto
    public float wheelBase = 0.16f;          // distanza tra le ruote (m)
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
        Vector3 currentPos = transform.position;

        Vector3 dir = new Vector3(target.x - currentPos.x, 0f, target.z - currentPos.z);
        float distance = dir.magnitude;
        dir.Normalize();

        // IMPORTANTE per TurtleBot3: usa transform.right come "forward"
        float angleToTarget = Vector3.SignedAngle(transform.right, dir, Vector3.up) * Mathf.Deg2Rad;

        // STEP 1 — Se siamo vicino al punto passa al prossimo
        if (distance <= reachThreshold)
        {
            currentIndex++;
            if (currentIndex >= path.Count)
            {
                StopWheels();
                isMoving = false;
                Debug.Log("Path completato!");
                return;
            }

            // nuovo punto = ricomincia con rotazione
            state = MoveState.Rotating;
            return;
        }

        // STEP 2 — Ruota fino ad allinearti
        float angleDegrees = Mathf.Abs(angleToTarget * Mathf.Rad2Deg);//TODO UNDERSTAND WHY ANGLE DEGREES DO NOT CHANGE EVEN IF THE ROBOT ROTEATE

        /*if (state == MoveState.Rotating)
        {
            Debug.Log("Angle Degrees: "+ angleDegrees);
            if (angleDegrees > 10f)
            {
                Debug.Log("STATE: Rotating. Angle to target: "+ angleToTarget+ "Angular speed: "+ angularSpeed);
                
                // Rotazione sul posto
                float w = Mathf.Clamp(angleToTarget * 2f, -angularSpeed, angularSpeed);

                //float leftSpeed = -w * (wheelBase / 2f);
                //float rightSpeed = w * (wheelBase / 2f);
                float leftSpeed = -w * (wheelBase * 6);
                float rightSpeed = w * (wheelBase * 6);
                Debug.Log("Wheels speed. : leftSpeed: " + leftSpeed + "rightSpeed: " + rightSpeed);
                SetWheelTarget(leftWheel, leftSpeed);
                SetWheelTarget(rightWheel, rightSpeed);
            }
            else
            {
                Debug.Log("STATE: Moving");
                // Allineato inizia a muoverti
                state = MoveState.Moving;
            }
            return;
        }*/
        if (state == MoveState.Rotating)
        {
            if (Mathf.Abs(angleToTarget * Mathf.Rad2Deg) > 5f)
            {
                float w = Mathf.Clamp(angleToTarget * 2f, -angularSpeed, angularSpeed);
                float leftSpeed = -w * (wheelBase / 2f);
                float rightSpeed = w * (wheelBase / 2f);

                SetWheelTarget(leftWheel, leftSpeed);
                SetWheelTarget(rightWheel, rightSpeed);
            }
            else
            {
                state = MoveState.Moving;
            }
            return;
        }
        



        // STEP 3 — Movimento verso il target
        float v = linearSpeed;
        float wMove = Mathf.Clamp(angleToTarget * 2f, -angularSpeed, angularSpeed);

        float left = v - wMove * (wheelBase / 2f);
        float right = v + wMove * (wheelBase / 2f);

        SetWheelTarget(leftWheel, left);
        SetWheelTarget(rightWheel, right);
    }

    void SetWheelTarget(ArticulationBody wheel, float speed)
    {
        // speed = velocità lineare della ruota in m/s
        float wheelRadPerSec = speed / wheelRadius; // converti in rad/s
        ArticulationDrive drive = wheel.xDrive;
        drive.targetVelocity = wheelRadPerSec * Mathf.Rad2Deg; // targetVelocity in gradi/s
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

        Debug.Log($"Ricevuto nuovo path con {newPath.Count} punti");
    }

    private void OnRosPathReceived(PathMsg msg)
    {
        List<Vector3> newPath = new List<Vector3>();

        foreach (var poseStamped in msg.poses)
        {
            // Conversione coordinate ROS -> Unity
            newPath.Add(new Vector3(
                (float)poseStamped.pose.position.x,
                0f,
                (float)poseStamped.pose.position.y
            ));
        }

        SetPath(newPath);
    }
}
