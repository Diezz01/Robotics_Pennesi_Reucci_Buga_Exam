using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Nav;

public class ExplorerController : GenericRobotController
{
    [Header("Mission Behavior")]
    public bool continuousOperation = true; // If false, stops after one cycle

    // -----------------------------
    // PATH AND TARGET
    // -----------------------------
    private List<Vector3> excavationPointsList = new List<Vector3>();
    private Queue<Vector3> targetQueue = new Queue<Vector3>();

    // Start is called before the first frame update
    void Start()
    {
        robotId = "explorer";
        chargingStationPosition = new Vector3(12f, 0f, -38f);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseArrayMsg>(topicNameTarget);
        ros.Subscribe<PathMsg>(topicNamePath, OnRosPathReceived);
    }

    // Update is called once per frame
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
            if (batterySimulator.currentCharge >= chargedThreshold)
            {
                Debug.Log($"<color=lime>Battery charged to {batterySimulator.currentCharge:F1}%. Resuming mission.</color>");
                StartNextTarget();
            }
        }

        if (waitingForPath) return;
        if (currentPath.Count == 0 || !isMoving) return;

        MoveAlongPathWithRotation();
    }

    // -----------------------------
    // LOAD EXCAVATION POINTS
    // -----------------------------
    private void GetExcavationPoints()
    {
        GameObject[] trovati = GameObject.FindGameObjectsWithTag("ExcavationPoint");

        foreach (GameObject go in trovati)
        {
            Vector3 pos = go.transform.position;
            excavationPointsList.Add(pos);
            targetQueue.Enqueue(pos);
            Debug.Log("Excavation point added: " + pos);
        }

        Debug.Log($"<color=green>Loaded {excavationPointsList.Count} excavation points for continuous mission cycle.</color>");
    }

    // -----------------------------
    // RELOAD EXCAVATION POINTS FOR CONTINUOUS CYCLE
    // -----------------------------
    private void ReloadMissionQueue()
    {
        // Clear the queue and reload excavation points for continuous operation
        targetQueue.Clear();

        foreach (Vector3 point in excavationPointsList)
        {
            targetQueue.Enqueue(point);
        }

        Debug.Log($"<color=green>Reloaded {targetQueue.Count} excavation points. Starting new mission cycle.</color>");
    }

    // -----------------------------
    // TARGET REACHED
    // -----------------------------
    protected override void OnReachedTarget()
    {
        Debug.Log($"<color=cyan>Target reached: {currentTarget}</color>");

        currentPath.Clear();
        currentPathIndex = 0;

        if (isChargingMission)
        {
            Debug.Log($"<color=yellow>Arrived at charging station. Current battery: {batterySimulator.currentCharge:F1}%. Waiting for charge...</color>");
            // Charging completion will be detected in Update()
            return;
        }

        // Normal mission point reached - continue to next target
        StartNextTarget();
    }

    // -----------------------------
    // NEXT TARGET SELECTION
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
        }

        // If queue is empty, reload excavation points for continuous operation
        if (targetQueue.Count == 0)
        {
            if (excavationPointsList.Count > 0 && continuousOperation)
            {
                Debug.Log("<color=lime>Mission cycle complete! Reloading excavation points for continuous operation.</color>");
                ReloadMissionQueue();
            }
            else if (!continuousOperation)
            {
                Debug.Log("<color=white>Mission cycle complete. Continuous operation disabled - robot stopped.</color>");
                return;
            }
            else
            {
                Debug.Log("<color=white>No excavation points available.</color>");
                return;
            }
        }

        Vector3 nextTarget = targetQueue.Peek();

        // Calculate cost to next target
        float distanceToTarget = Vector3.Distance(transform.position, nextTarget);
        float costToTarget = distanceToTarget * batterySimulator.dischargePerMeter * safetyCostMultiplier;

        // Calculate cost to return to charging station from next target
        float distanceToChargingFromTarget = Vector3.Distance(nextTarget, chargingStationPosition);
        float costToChargingFromTarget = distanceToChargingFromTarget * batterySimulator.dischargePerMeter * safetyCostMultiplier;

        // Total cost: go to target + return to charging
        float totalCost = costToTarget + costToChargingFromTarget;

        // Check if we have enough battery for mission + return to charging
        if (batterySimulator.currentCharge >= totalCost)
        {
            // Sufficient battery - proceed with mission
            currentTarget = targetQueue.Dequeue();
            isChargingMission = false;
            PublishTarget(currentTarget);

            Debug.Log($"<color=cyan>Starting mission to {currentTarget}. Battery: {batterySimulator.currentCharge:F1}% | Cost: {totalCost:F1}%</color>");
        }
        else
        {
            // Insufficient battery - check if we can even reach charging station
            float distanceToCharging = Vector3.Distance(transform.position, chargingStationPosition);
            float costToCharging = distanceToCharging * batterySimulator.dischargePerMeter * safetyCostMultiplier;

            if (batterySimulator.currentCharge < costToCharging)
            {
                // CRITICAL: Cannot reach charging station
                Debug.LogError($"<color=red>CRITICAL: Battery too low to reach charging station! Battery: {batterySimulator.currentCharge:F1}% | Required: {costToCharging:F1}%</color>");
                // Stop all operations - robot is stranded
                return;
            }

            // Insert charging mission (only if not already inserted)
            if (!hasInsertedChargingMission)
            {
                Debug.Log($"<color=yellow>Insufficient battery for mission. Inserting charging station visit. Battery: {batterySimulator.currentCharge:F1}% | Required: {totalCost:F1}%</color>");
                hasInsertedChargingMission = true;
            }

            isChargingMission = true;
            isReturningToBase = true;
            currentTarget = chargingStationPosition;
            PublishTarget(chargingStationPosition);
        }
    }

}
