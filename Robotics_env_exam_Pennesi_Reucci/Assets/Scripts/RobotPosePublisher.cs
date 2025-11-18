using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class RobotPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string poseTopic = "/robot_pose"; // ora publish pose (position + orientation)

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        //Invoke(nameof(RegisterPublisher), 0.1f);
        RegisterPublisher();
    }

    void RegisterPublisher()
    {
        ros.RegisterPublisher<PoseMsg>(poseTopic);
    }
   
    void Update()
    {
        float yawRad = transform.eulerAngles.y * Mathf.Deg2Rad;
        float halfYaw = yawRad / 2.0f;

        QuaternionMsg q = new QuaternionMsg(0, 0, Mathf.Sin(halfYaw), Mathf.Cos(halfYaw));

        PoseMsg pose = new PoseMsg
        {
            position = new PointMsg(transform.position.x, transform.position.z, transform.position.y),
            orientation = q
        };

        ros.Publish(poseTopic, pose);
    }



}
