using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static UnityEditor.PlayerSettings;

public class ExplorerController : MonoBehaviour
{
    // -----------------------------
    // CONFIG
    // -----------------------------
    private float batteryLevel = 100.0f;
    private float batteryConsume = 0.5f; // battery consume per meter
    private Vector3 explorerBase = new Vector3(12f, 0f, -38f);

    public float linearSpeed = 4.0f;       // velocità robot
    public float angularSpeed = 180f;     // gradi/sec
    public float reachThreshold = 0.01f;  // distanza minima per considerare punto raggiunto

    public string topicNameTarget = "/target";
    public string topicNamePath = "/astar_path";

    // -----------------------------
    // PATH E TARGET
    // -----------------------------
    private List<Vector3> excavationPointsList = new List<Vector3>();
    private Queue<Vector3> targetQueue = new Queue<Vector3>();

    private List<Vector3> currentPath = new List<Vector3>();
    private int currentPathIndex = 0;
    private Vector3 currentTarget;

    private bool waitingForPath = false;
    private bool isReturningToBase = false;

    private bool isMoving = false;
    private MoveState state = MoveState.Rotating;

    ROSConnection ros;

    private enum MoveState { Rotating, Moving }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseArrayMsg>(topicNameTarget);
        ros.Subscribe<PathMsg>(topicNamePath, OnRosPathReceived);
    }

    void Update()
    {
        if (excavationPointsList.Count == 0)
        {
            GetExcavationPoints();
            StartNextTarget();
        }

        if (waitingForPath) return;
        if (currentPath.Count == 0 || !isMoving) return;

        MoveAlongPathWithRotation();
    }

    // -----------------------------
    // CARICO I PUNTI DI SCAVO
    // -----------------------------
    private void GetExcavationPoints()
    {
        GameObject[] trovati = GameObject.FindGameObjectsWithTag("ExcavationPoint");

        foreach (GameObject go in trovati)
        {
            Vector3 pos = go.transform.position;
            excavationPointsList.Add(pos);
            targetQueue.Enqueue(pos);
            Debug.Log("Punto di scavo aggiunto: " + pos);
        }
        excavationPointsList.Add(explorerBase);
        targetQueue.Enqueue(explorerBase);
    }

    // -----------------------------
    // PUBBLICO IL TARGET SU ROS
    // -----------------------------
    private void PublishTarget(Vector3 target)
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

    // -----------------------------
    // CALLBACK PATH DA ROS
    // -----------------------------
    private void OnRosPathReceived(PathMsg msg)
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
        state = MoveState.Rotating;

        Debug.Log($"<color=green>Path ricevuto da ROS. Lunghezza: {currentPath.Count}</color>");
    }

    // -----------------------------
    // MOVIMENTO CON ROTAZIONE
    // -----------------------------
    private void MoveAlongPathWithRotation()
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
            state = MoveState.Rotating;
            return;
        }

        // Calcolo angolo verso il target
        float angleToTarget = Vector3.SignedAngle(transform.forward, dirNorm, Vector3.up);

        // ------- ROTAZIONE -------
        if (state == MoveState.Rotating)
        {
            if (Mathf.Abs(angleToTarget) > 2f)
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
        if (state == MoveState.Moving)
        {
            transform.position += transform.forward * linearSpeed * Time.deltaTime;
        }
    }

    // -----------------------------
    // TARGET RAGGIUNTO
    // -----------------------------
    private void OnReachedTarget()
    {
        Debug.Log($"<color=cyan>Target raggiunto: {currentTarget}</color>");

        float distance = Vector3.Distance(transform.position, currentTarget);
        batteryLevel -= distance * batteryConsume;
        Debug.Log($"<color=orange>Batteria attuale: {batteryLevel}%</color>");

        currentPath.Clear();
        currentPathIndex = 0;

        if (isReturningToBase)
        {
            Debug.Log("<color=lime>Tornato alla base: batteria ricaricata.</color>");
            batteryLevel = 100f;
            isReturningToBase = false;

            StartNextTarget();
            return;
        }

        StartNextTarget();
    }

    // -----------------------------
    // SELEZIONE PROSSIMO TARGET
    // -----------------------------
    private void StartNextTarget()
    {
        if (targetQueue.Count == 0)
        {
            Debug.Log("<color=white>Nessun altro punto da visitare.</color>");
            return;
        }

        Vector3 nextTarget = targetQueue.Peek();
        float expectedCost = Vector3.Distance(transform.position, nextTarget) * batteryConsume;

        if (batteryLevel >= expectedCost)
        {
            currentTarget = targetQueue.Dequeue();
            PublishTarget(currentTarget);
        }
        else
        {
            Debug.Log("<color=red>Batteria insufficiente: torno alla base.</color>");
            isReturningToBase = true;
            currentTarget = explorerBase;
            PublishTarget(explorerBase);
        }
    }
}
