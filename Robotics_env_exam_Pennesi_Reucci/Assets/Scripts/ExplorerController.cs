using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using System;

public class ExplorerController : GenericRobotController
{
    [Header("Mission Behavior")]
    public bool continuousOperation = true; // If false, stops after one cycle

    // -----------------------------
    // PATH AND TARGET
    // -----------------------------
    private List<MapTarget> excavationPointsList = new List<MapTarget>();
    private Queue<MapTarget> targetQueue = new Queue<MapTarget>();
    private MapTarget chargingStation; 

    // Start is called before the first frame update
    void Start()
    {
        // Start staggered initialization to prevent ros_tcp_endpoint overload
        StartCoroutine(InitializeWithConnectionCheck());
    }

    IEnumerator InitializeWithConnectionCheck()
    {
        // Setup robot ID
        if (string.IsNullOrEmpty(robotId))
        {
            robotId = $"tb3_{robotIndex}";
        }

        // CRITICAL: Stagger initialization by robot index to prevent endpoint crash
        // With 6 robots registering 4+ topics each (24+ total), the endpoint crashes
        // Staggering prevents simultaneous topic registration overload
        float staggerDelay = robotIndex * 0.5f;
        if (staggerDelay > 0)
        {
            Debug.Log($"<color=yellow>{robotId}: Waiting {staggerDelay}s before initialization (stagger to prevent endpoint crash)</color>");
            yield return new WaitForSeconds(staggerDelay);
        }

        // Setup topic names
        topicNameTarget = $"/tb3_{robotIndex}/target";
        topicNamePath = $"/tb3_{robotIndex}/astar_path";
        topicNamePose = $"/tb3_{robotIndex}/pose";
        topicNameCollision = $"/tb3_{robotIndex}/collision_detected";

        ros = ROSConnection.GetOrCreateInstance();

        // Wait for ROS connection to be established
        int maxRetries = 20;  // 20 retries × 0.5s = 10 seconds max wait
        int retries = 0;
        while (!ros.HasConnectionThread && retries < maxRetries)
        {
            if (retries == 0)
            {
                Debug.Log($"<color=yellow>{robotId}: Waiting for ROS connection...</color>");
            }
            yield return new WaitForSeconds(0.5f);
            retries++;
        }

        if (!ros.HasConnectionThread)
        {
            Debug.LogError($"<color=red>{robotId}: Failed to connect to ROS after {maxRetries * 0.5f} seconds. Check ROS TCP endpoint is running!</color>");
            yield break;
        }

        Debug.Log($"<color=lime>{robotId}: ROS connected! Registering topics...</color>");

        // Register publishers
        ros.RegisterPublisher<PoseArrayMsg>(topicNameTarget);
        ros.RegisterPublisher<PoseStampedMsg>(topicNamePose);

        // Register subscribers
        ros.Subscribe<PathMsg>(topicNamePath, OnRosPathReceived);
        ros.Subscribe<BoolMsg>(topicNameCollision, OnCollisionDetected);

        Debug.Log($"<color=cyan>Robot {robotId} topics registered: {topicNameTarget}, {topicNamePath}, {topicNamePose}, {topicNameCollision}</color>");

        // Setup charging station
        chargingStationPosition = new Vector3(12f, 0f, -38f);
        chargingStation = new Target(chargingStationPosition);

        // Mark initialization as complete
        isFullyInitialized = true;
        Debug.Log($"<color=lime>{robotId}: Initialization complete - ready to publish!</color>");
    }

    private bool hasStartedMission = false;

    // Update is called once per frame
    void Update()
    {
        // Continuously publish position for collision avoidance
        UpdatePositionPublishing();

        if (excavationPointsList.Count == 0)
        {
            GetExcavationPoints();
            // Don't start immediately - wait a bit for map to be published
            if (!hasStartedMission)
            {
                StartCoroutine(DelayedMissionStart());
                hasStartedMission = true;
            }
            return;
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
    // DELAYED MISSION START
    // -----------------------------
    IEnumerator DelayedMissionStart()
    {
        // Wait for map to be generated and published
        Debug.Log($"<color=yellow>Robot {robotId}: Waiting for map to be published...</color>");
        yield return new WaitForSeconds(2f); // Wait 2 seconds for map

        Debug.Log($"<color=lime>Robot {robotId}: Starting navigation!</color>");
        StartNextTarget();
    }

    // -----------------------------
    // LOAD EXCAVATION POINTS
    // -----------------------------
    private void GetExcavationPoints()
    {
        GameObject[] trovati = GameObject.FindGameObjectsWithTag("ExcavationPoint");

        foreach (GameObject go in trovati)
        {
            ExcavationPoint component = go.GetComponent<ExcavationPoint>();

            // Create data wrapper for navigation
            ExcavationPointTarget target = new ExcavationPointTarget(
                go.transform.position,
                component.Type
            );

            excavationPointsList.Add(target);
            targetQueue.Enqueue(target);
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

        foreach (MapTarget point in excavationPointsList)
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

        // Log excavation point type if it's an excavation target
        if (currentTarget is ExcavationPointTarget excPoint)
        {
            Debug.Log($"<color=cyan>Excavation Point type: {excPoint.Type}</color>");
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

        MapTarget nextTarget = targetQueue.Peek();
        Vector3 positionNextTarget = nextTarget.Position;

        // Calculate cost to next target
        float distanceToTarget = Vector3.Distance(transform.position, positionNextTarget);
        float costToTarget = distanceToTarget * batterySimulator.dischargePerMeter * safetyCostMultiplier;

        // Calculate cost to return to charging station from next target
        float distanceToChargingFromTarget = Vector3.Distance(positionNextTarget, chargingStationPosition);
        float costToChargingFromTarget = distanceToChargingFromTarget * batterySimulator.dischargePerMeter * safetyCostMultiplier;

        // Total cost: go to target + return to charging
        float totalCost = costToTarget + costToChargingFromTarget;

        // Check if we have enough battery for mission + return to charging
        if (batterySimulator.currentCharge >= totalCost)
        {
            // Sufficient battery - proceed with mission
            currentTarget = targetQueue.Dequeue();
            isChargingMission = false;
            Vector3 positionCurrentTarget = currentTarget.Position;
            PublishTarget(positionCurrentTarget);

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
            currentTarget = chargingStation;
            PublishTarget(chargingStationPosition);
        }
    }

}
