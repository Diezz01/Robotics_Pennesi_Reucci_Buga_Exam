using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;

public class PathVisualizerROS : MonoBehaviour
{
    public string topicName = "/astar_path";
    public LineRenderer lineRenderer;
    ROSConnection ros;

    void Start()
    {
        // Auto-detect robot index from parent ExplorerController
        ExplorerController controller = GetComponentInParent<ExplorerController>();
        if (controller != null)
        {
            topicName = $"/tb3_{controller.robotIndex}/astar_path";
            Debug.Log($"<color=cyan>PathVisualizer subscribing to {topicName}</color>");
        }
        else
        {
            Debug.LogWarning("PathVisualizer: Could not find ExplorerController. Using default topic.");
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathMsg>(topicName, ReceivePath);
    }

    /*
    void ReceivePath(PathMsg pathMsg)
    {
        if (pathMsg.poses.Length == 0) return;

        lineRenderer.positionCount = pathMsg.poses.Length;
        for (int i = 0; i < pathMsg.poses.Length; i++)
        {
            var p = pathMsg.poses[i].pose.position;
            lineRenderer.SetPosition(i, new Vector3((float)p.x, 0.1f, (float)p.z));
        }
    }
    */

    void ReceivePath(PathMsg msg)
    {
        if (msg.poses == null || msg.poses.Length == 0)
        {
            Debug.LogWarning("Empty path received.");
            return;
        }

        if (lineRenderer == null)
        {
            Debug.LogError("LineRenderer NOT assigned in the inspector!");
            return;
        }

        lineRenderer.positionCount = msg.poses.Length;

        for (int i = 0; i < msg.poses.Length; i++)
        {
            var pose = msg.poses[i].pose;

            // ROS = X (forward), Y (up), Z (left)
            // Unity = X (right), Y (up), Z (forward)
            Vector3 posUnity = new Vector3(
                (float)pose.position.x,
                (float)pose.position.z,
                (float)pose.position.y
            );

            lineRenderer.SetPosition(i, posUnity);

            //Debug.Log($"POINT {i}: ({posUnity.x}, {posUnity.z})");
        }
    }
}
