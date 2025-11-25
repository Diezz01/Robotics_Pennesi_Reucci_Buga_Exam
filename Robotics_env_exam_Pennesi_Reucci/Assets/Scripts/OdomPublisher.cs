using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class OdometryPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/odom";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);
    }

    void LateUpdate()
    {
        OdometryMsg odom = new OdometryMsg();

        // Header
        HeaderMsg header = new HeaderMsg();
        double t = Time.realtimeSinceStartup;
        uint secs = (uint)Mathf.Floor((float)t);
        uint nsecs = (uint)((t - secs) * 1e9f);
        header.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg((int)secs, nsecs);
        header.frame_id = "odom";
        odom.header = header;

        // posizione
        Vector3 pos = transform.position;
        odom.pose.pose.position = new PointMsg(pos.x, pos.z, pos.y);
        //Debug.Log("Position x: " + pos.x + "z: "+pos.z+"y: "+pos.y);
        // orientazione
        Quaternion rot = transform.rotation;
        odom.pose.pose.orientation = new QuaternionMsg(rot.x, rot.y, rot.z, rot.w);

        // velocità zero (opzionale)
        odom.twist.twist.linear = new Vector3Msg(0, 0, 0);
        odom.twist.twist.angular = new Vector3Msg(0, 0, 0);

        ros.Publish(topicName, odom);
    }
}
