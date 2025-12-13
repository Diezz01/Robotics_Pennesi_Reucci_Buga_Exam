using UnityEngine;
using RosMessageTypes.RobotInterfaces;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class ExcavatorController : GenericRobotController
{
    [Header("ROS Settings")]
    public string excavationTopicName = "excavationRequest";

    private bool hasMission = false;
    
    public enum ExcavationRobotType
    {
        Excavation = 0,
        Analysys = 1,
        GasAnalysys = 2
    }

    public ExcavationRobotType RobotType;
    public void Initialize(string id)
    {
        robotId = id;
    }
    void Start()
    {
        if (robotId == "")
            return;

        ros = ROSConnection.GetOrCreateInstance();

        ros.Subscribe<PathMsg>($"/{robotId}{topicNamePath}", OnRosPathReceived);
        ros.RegisterPublisher<PoseArrayMsg>($"/{robotId}{topicNameTarget}");
        ros.Subscribe<ExcavationPointMsg>(excavationTopicName, OnExcavationPointReceived);

        //Debug.Log($"<color=yellow>ExcavatorController subscribed to {baseTopic}{excavationTopicName}</color>");
        //Debug.Log($"<color=yellow>ExcavatorController subscribed to {baseTopic}{topicNamePath}</color>");
    }

    void Update()
    {
        // Check if charging is complete

        if (!hasMission) return;

        //if (waitingForPath) return;
        //if (currentPath.Count == 0 || !isMoving) return;
        MoveAlongPathWithRotation();
    }
    
    private void OnExcavationPointReceived(ExcavationPointMsg msg)
    {
        Debug.Log("<color=cyan>Received ExcavationPointMsg</color>");

        Debug.Log($"-> ID: {msg.id}");
        Debug.Log($"-> Type: {msg.type}");
        Debug.Log($"-> Position: ({msg.x}, {msg.y}, {msg.z})");

        ExcavationPoint.ExcavationType receivedType = (ExcavationPoint.ExcavationType) msg.type;
        
        if ((ExcavationRobotType)receivedType != RobotType) // If excavation point and robot does not have the same type -> do nothing
            return;
        
        Vector3 targetPos = new Vector3((float)msg.x, (float)msg.y, (float)msg.z);

        Debug.Log($"<color=green>Mission accepted: This robot handles this excavation type. {targetPos.x} {targetPos.y} {targetPos.z} </color>");
    
        //currentTarget = new Target(targetPos);
        hasMission = true;
        //isChargingMission = false;

        PublishTarget(targetPos);
    }

    private void ReturnToBase()
    {
        PublishTarget(chargingStationPosition);
    }

    protected override void OnReachedTarget()
    {
        ReturnToBase();
        Debug.Log($"<color=green>Target reached.</color>");
    }
}
