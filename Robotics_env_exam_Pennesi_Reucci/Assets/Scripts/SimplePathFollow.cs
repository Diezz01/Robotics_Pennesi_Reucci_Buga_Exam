using RosMessageTypes.Nav;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SimplePathFollower : MonoBehaviour
{
    [Header("Movement")]
    public float linearSpeed = 1.0f;
    public float angularSpeed = 180f;     // gradi al secondo
    public float reachThreshold = 0.05f;

    private enum MoveState { Rotating, Moving }
    private MoveState state = MoveState.Rotating;

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


    void Update()
    {
        if (!isMoving || path.Count == 0) return;

        Vector3 target = path[currentIndex];
        Vector3 dir = new Vector3(
            target.x - transform.position.x,
            0f,
            target.z - transform.position.z
        );

        float distance = dir.magnitude;
        Vector3 dirNorm = dir.normalized;

        // Se arrivato al punto  passa al successivo
        if (distance <= reachThreshold)
        {
            currentIndex++;
            if (currentIndex >= path.Count)
            {
                Debug.Log("Path completato!");
                isMoving = false;
                return;
            }
            state = MoveState.Rotating;
            return;
        }

        // Angolo verso il target
        float angleToTarget = Vector3.SignedAngle(transform.forward, dirNorm, Vector3.up);

        // ------- ROTAZIONE -------
        if (state == MoveState.Rotating)
        {
            if (Mathf.Abs(angleToTarget) > 2f) // rotazione precisa entro 2°
            {
                float rotateStep = Mathf.Sign(angleToTarget) * angularSpeed * Time.deltaTime;
                rotateStep = Mathf.Clamp(rotateStep, -Mathf.Abs(angleToTarget), Mathf.Abs(angleToTarget));

                transform.Rotate(0f, rotateStep, 0f);
            }
            else
            {
                state = MoveState.Moving;
            }
            return;
        }

        // ------- MOVIMENTO -------
        transform.position += transform.forward * linearSpeed * Time.deltaTime;
    }



    public void SetPath(List<Vector3> newPath)
    {
        path = newPath;
        currentIndex = 0;
        isMoving = true;
        state = MoveState.Rotating;

        Debug.Log($"Ricevuto nuovo path con {newPath.Count} punti");
    }


    private void OnRosPathReceived(PathMsg msg)
    {
        List<Vector3> newPath = new List<Vector3>();

        foreach (var poseStamped in msg.poses)
        {
            newPath.Add(new Vector3(
                (float)poseStamped.pose.position.x,
                0f,
                (float)poseStamped.pose.position.y
            ));
        }

        SetPath(newPath);
    }
}
