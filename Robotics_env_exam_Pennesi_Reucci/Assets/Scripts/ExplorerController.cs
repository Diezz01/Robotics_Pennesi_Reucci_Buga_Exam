using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Nav;

public class ExplorerController : GenericRobotController
{
    private Vector3 explorerBase = new Vector3(12f, 0f, -38f);
    private List<Vector3> excavationPointsList = new List<Vector3>();
    private Queue<Vector3> targetQueue = new Queue<Vector3>();

    private float batteryLevel = 100.0f;
    private float batteryConsume = 0.5f; // battery consume per meter

    // Start is called before the first frame update
    void Start()
    {
        robotId = "explorer";
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

        if (waitingForPath) return;
        if (currentPath.Count == 0 || !isMoving) return;

        MoveAlongPathWithRotation();
    }

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
        excavationPointsList.Add(explorerBase);
        targetQueue.Enqueue(explorerBase);
    }

    protected override void OnReachedTarget()
    {
        Debug.Log($"<color=cyan>Target reached: {currentTarget}</color>");

        float distance = Vector3.Distance(transform.position, currentTarget);
        batteryLevel -= distance * batteryConsume;
        Debug.Log($"<color=orange>Actual battery Level: {batteryLevel}%</color>");

        currentPath.Clear();
        currentPathIndex = 0;

        if (isReturningToBase)
        {
            Debug.Log("<color=lime>Back to base: battery charged</color>");
            batteryLevel = 100f;
            isReturningToBase = false;

            StartNextTarget();
            return;
        }

        StartNextTarget();
    }

    private void StartNextTarget()
    {
        if (targetQueue.Count == 0)
        {
            Debug.Log("<color=white>All point are visited</color>");
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
            Debug.Log("<color=red>Low battery Level: back to the base</color>");
            isReturningToBase = true;
            currentTarget = explorerBase;
            PublishTarget(explorerBase);
        }
    }

}
