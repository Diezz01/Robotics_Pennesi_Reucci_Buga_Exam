using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class BatterySimulator : MonoBehaviour
{
    [Header("ROS")]
    public string batteryTopic = "/tb3_0/battery_state";
    public string chargingStateTopic = "/tb3_0/charging_status";

    [Header("Battery Settings")]
    public float maxCharge = 100f;
    public float currentCharge = 100f;
    public float dischargePerMeter = 0.5f;      // how much battery per meter
    public float dischargePerSecondIdle = 0.01f; // idle drain
    public float chargePerSecond = 5f;          // charging speed

    private ROSConnection ros;
    private Vector3 lastPosition;
    private bool isCharging = false;
    private bool wasCharging = false;

    void Start()
    {
        ros = ROSConnection.instance;
        lastPosition = transform.position;

        // Make sure the ROSConnection inspector has a publisher for this topic:
        // Topic Name: /tb3_0/battery_state
        // Message Type: std_msgs/Float32
    }

    void Update()
    {
        float dt = Time.deltaTime;

        // Distance moved since last frame
        float distance = Vector3.Distance(transform.position, lastPosition);
        lastPosition = transform.position;

        // Battery logic: charge if in zone, drain otherwise
        if (isCharging)
        {
            currentCharge += chargePerSecond * dt;
        }
        else
        {
            currentCharge -= distance * dischargePerMeter;
            currentCharge -= dischargePerSecondIdle * dt;
        }

        currentCharge = Mathf.Clamp(currentCharge, 0f, maxCharge);

        // Publish battery level to ROS
        Float32Msg batteryMsg = new Float32Msg(currentCharge);
        ros.Publish(batteryTopic, batteryMsg);

        // Publish charging state change
        if (isCharging != wasCharging)
        {
            BoolMsg chargingStatusMsg = new BoolMsg(isCharging);
            ros.Publish(chargingStateTopic, chargingStatusMsg);
            wasCharging = isCharging;

            Debug.Log($"<color=magenta>Charging status: {isCharging} | Battery: {currentCharge:F1}%</color>");
        }

        // Warning for critical battery
        if (currentCharge <= 10f && !isCharging)
        {
            Debug.LogWarning($"<color=red>CRITICAL BATTERY: {currentCharge:F1}%</color>");
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("ChargingZone"))
        {
            isCharging = true;
            // Optional: stop robot movement while charging (set cmd_vel to zero etc.)
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("ChargingZone"))
        {
            isCharging = false;
        }
    }
}
