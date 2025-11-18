using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string cmdVelTopic = "/cmd_vel";

    private float linearX = 0f;
    private float angularZ = 0f;

    public float linearScale = 1.0f;
    public float angularScale = 1.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(cmdVelTopic, ReceiveCmdVel);
    }

    void ReceiveCmdVel(TwistMsg msg)
    {
        linearX = (float)msg.linear.x;
        angularZ = (float)msg.angular.z;
    }

    void FixedUpdate()
    {
        // Applica rotazione (angularZ in rad/s)
        float deltaDegrees = angularZ * Mathf.Rad2Deg * Time.fixedDeltaTime;
        transform.Rotate(Vector3.up, deltaDegrees);

        // Applica movimento in avanti lungo forward attuale
        Vector3 move = transform.forward * linearX * linearScale * Time.fixedDeltaTime;
        transform.position += move;
    }
}
