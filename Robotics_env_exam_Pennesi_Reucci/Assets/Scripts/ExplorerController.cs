using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static UnityEditor.PlayerSettings;

public class ExplorerController : MonoBehaviour
{
    // -----------------------------
    // CONFIG
    // -----------------------------
    private Vector3 explorerBase = new Vector3(12f, 0f, -38f);

    [Header("Battery Management")]
    public Vector3 chargingStationPosition = new Vector3(12f, 0f, -38f);
    public float batteryDischargePerMeter = 0.5f;
    public float lowBatteryThreshold = 30f;
    public float chargedThreshold = 95f;
    public float safetyCostMultiplier = 1.2f;
    public string batteryTopicName = "/tb3_0/battery_state";

    private float currentBatteryLevel = 100f;
    private bool isChargingMission = false;
    private bool hasInsertedChargingMission = false;

    public float linearSpeed = 4.0f;       // velocit� robot
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

        // Subscribe to battery state from BatterySimulator
        ros.Subscribe<Float32Msg>(batteryTopicName, OnBatteryStateReceived);
    }

    void Update()
    {
        if (excavationPointsList.Count == 0)
        {
            GetExcavationPoints();
            StartNextTarget();
        }

        // Check if charging is complete
        if (isChargingMission && !waitingForPath && !isMoving)
        {
            if (currentBatteryLevel >= chargedThreshold)
            {
                Debug.Log($"<color=lime>Battery charged to {currentBatteryLevel:F1}%. Resuming mission.</color>");
                StartNextTarget();
            }
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
    // CALLBACK BATTERY STATE
    // -----------------------------
    private void OnBatteryStateReceived(Float32Msg msg)
    {
        currentBatteryLevel = msg.data;

        // Debug: Log battery updates periodically (every 5%)
        if (Mathf.Abs(currentBatteryLevel % 5) < 0.1f)
        {
            Debug.Log($"<color=cyan>ExplorerController: Battery level received: {currentBatteryLevel:F1}%</color>");
        }
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

        // Controllo se il punto � raggiunto
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

        currentPath.Clear();
        currentPathIndex = 0;

        if (isChargingMission)
        {
            Debug.Log($"<color=yellow>Arrived at charging station. Current battery: {currentBatteryLevel:F1}%. Waiting for charge...</color>");
            // Charging completion will be detected in Update()
            return;
        }

        // Normal mission point reached - continue to next target
        StartNextTarget();
    }

    // -----------------------------
    // SELEZIONE PROSSIMO TARGET
    // -----------------------------
    private void StartNextTarget()
    {
        // If we just finished charging, reset the flag and continue
        if (isChargingMission)
        {
            Debug.Log("<color=lime>Charging complete! Resuming mission queue.</color>");
            isChargingMission = false;
            hasInsertedChargingMission = false;
            isReturningToBase = false;

            // Check if there are remaining targets
            if (targetQueue.Count == 0)
            {
                Debug.Log("<color=white>All missions completed!</color>");
                return;
            }
        }

        if (targetQueue.Count == 0)
        {
            Debug.Log("<color=white>No more points to visit.</color>");
            return;
        }

        Vector3 nextTarget = targetQueue.Peek();

        // Calculate cost to next target
        float distanceToTarget = Vector3.Distance(transform.position, nextTarget);
        float costToTarget = distanceToTarget * batteryDischargePerMeter * safetyCostMultiplier;

        // Calculate cost to return to charging station from next target
        float distanceToChargingFromTarget = Vector3.Distance(nextTarget, chargingStationPosition);
        float costToChargingFromTarget = distanceToChargingFromTarget * batteryDischargePerMeter * safetyCostMultiplier;

        // Total cost: go to target + return to charging
        float totalCost = costToTarget + costToChargingFromTarget;

        // Check if we have enough battery for mission + return to charging
        if (currentBatteryLevel >= totalCost)
        {
            // Sufficient battery - proceed with mission
            currentTarget = targetQueue.Dequeue();
            isChargingMission = false;
            PublishTarget(currentTarget);

            Debug.Log($"<color=cyan>Starting mission to {currentTarget}. Battery: {currentBatteryLevel:F1}% | Cost: {totalCost:F1}%</color>");
        }
        else
        {
            // Insufficient battery - check if we can even reach charging station
            float distanceToCharging = Vector3.Distance(transform.position, chargingStationPosition);
            float costToCharging = distanceToCharging * batteryDischargePerMeter * safetyCostMultiplier;

            if (currentBatteryLevel < costToCharging)
            {
                // CRITICAL: Cannot reach charging station
                Debug.LogError($"<color=red>CRITICAL: Battery too low to reach charging station! Battery: {currentBatteryLevel:F1}% | Required: {costToCharging:F1}%</color>");
                // Stop all operations - robot is stranded
                return;
            }

            // Insert charging mission (only if not already inserted)
            if (!hasInsertedChargingMission)
            {
                Debug.Log($"<color=yellow>Insufficient battery for mission. Inserting charging station visit. Battery: {currentBatteryLevel:F1}% | Required: {totalCost:F1}%</color>");
                hasInsertedChargingMission = true;
            }

            isChargingMission = true;
            isReturningToBase = true;
            currentTarget = chargingStationPosition;
            PublishTarget(chargingStationPosition);
        }
    }
}
