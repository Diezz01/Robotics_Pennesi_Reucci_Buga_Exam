using RosMessageTypes.Nav;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static UnityEditor.PlayerSettings;


public class OccupancyGridGenerator : MonoBehaviour
{

    [Header("Rock Prefabs")]
    [Tooltip("Array of rock prefabs to spawn")]
    public GameObject[] rockPrefabs;

    public GameObject excPointPrefab;

    public Transform excPointParent;

    [Header("Spawn Settings")]
    [Tooltip("Number of rocks to spawn")]
    public int numberOfRocks = 20;

    [Tooltip("Minimum position bounds for spawning")]
    public Vector3 minPosition = new Vector3(-50f, 0f, -50f);

    [Tooltip("Maximum position bounds for spawning")]
    public Vector3 maxPosition = new Vector3(50f, 0f, 50f);

    [Header("Rotation Settings")]
    [Tooltip("Minimum rotation for X axis (in degrees)")]
    public float minRotationX = 0f;

    [Tooltip("Maximum rotation for X axis (in degrees)")]
    public float maxRotationX = 360f;

    [Tooltip("Fixed Y axis rotation (in degrees)")]
    public float fixedRotationY = 0f;

    [Tooltip("Minimum rotation for Z axis (in degrees)")]
    public float minRotationZ = 0f;

    [Tooltip("Maximum rotation for Z axis (in degrees)")]
    public float maxRotationZ = 360f;

    [Header("Scale Settings")]
    [Tooltip("Enable random scale variation")]
    public bool randomizeScale = true;

    [Tooltip("Minimum scale multiplier")]
    public float minScale = 0.8f;

    [Tooltip("Maximum scale multiplier")]
    public float maxScale = 1.5f;

    [Header("Spawning Options")]
    [Tooltip("Spawn rocks on Start")]
    public bool spawnOnStart = true;

    [Tooltip("Parent spawned rocks under this transform")]
    public Transform rocksParent;
    //////////////


    public GameObject robotPrefab;     // TurtleBot3 prefab
    public Transform robotsParent;

    public int mapWidth = 100;   // celle
    public int mapHeight = 100;  // celle
    public float cellSize = 0.5f; // dimensione in metri
    public float heightCheck = 0.0f;
    public LayerMask obstacleLayer;

    public string fileName = "unity_map";
    public string topicName = "/map";

    private Vector3[] chargingPoints = new Vector3[]
    {
        new Vector3(42f, 0f, -38f),
        new Vector3(37f, 0f, -38f),
        new Vector3(32f, 0f, -38f),
        new Vector3(27f, 0f, -38f),
        new Vector3(22f, 0f, -38f),
        new Vector3(17f, 0f, -38f),
    };


    ROSConnection ros;
    OccupancyGridMsg msg;

    void Start()
    {

        int numRobots = Random.Range(1, chargingPoints.Length + 1);

        if (spawnOnStart)
        {
            SpawnRobots(numRobots);
            SpawnRocks();
            SpawnExcavationPoints(numRobots);
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OccupancyGridMsg>(topicName);

        int[,] grid = new int[mapWidth, mapHeight];

        Vector3 origin = transform.position - new Vector3(mapWidth, 0, mapHeight) * cellSize * 0.5f;

        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                Vector3 cellCenter = origin + new Vector3(x * cellSize + cellSize / 2, heightCheck / 2, y * cellSize + cellSize / 2);

                if (Physics.CheckBox(cellCenter, new Vector3(cellSize / 2, heightCheck / 2, cellSize / 2), Quaternion.identity, obstacleLayer))
                {
                    grid[x, y] = 1; // occupata
                }
                else
                {
                    grid[x, y] = 0;   // libera
                }
            }
        }

        SavePGM(grid);
        SaveYAML();
      //  Debug.Log("Occupancy Grid saved!");

        msg = new OccupancyGridMsg();
        msg.info.resolution = cellSize;
        msg.info.width = (uint)mapWidth;
        msg.info.height = (uint)mapHeight;
        msg.info.origin.position.x = origin.x;
        msg.info.origin.position.y = origin.z; // Unity Z -> ROS Y
        msg.info.origin.position.z = 0;
        msg.info.origin.orientation.w = 1.0f;

        msg.data = new sbyte[mapWidth * mapHeight];
        for (int y = 0; y < mapHeight; y++)
            for (int x = 0; x < mapWidth; x++)
                msg.data[y * mapWidth + x] = (sbyte)grid[x, y];

        ros.Publish(topicName, msg);
        Debug.Log("Map: Published");
    }

    void SavePGM(int[,] grid)
    {
        string path = Path.Combine(Application.dataPath, fileName + ".pgm");
        using (StreamWriter sw = new StreamWriter(path))
        {
            sw.WriteLine("P2");
            sw.WriteLine($"{mapWidth} {mapHeight}");
            sw.WriteLine("100");

            for (int y = mapHeight - 1; y >= 0; y--)
            {
                for (int x = 0; x < mapWidth; x++)
                {
                    sw.Write(grid[x, y] + " ");
                }
                sw.WriteLine();
            }
        }
    }

    void SaveYAML()
    {
        string path = Path.Combine(Application.dataPath, fileName + ".yaml");
        using (StreamWriter sw = new StreamWriter(path))
        {
            sw.WriteLine("image: " + fileName + ".pgm");
            sw.WriteLine("resolution: " + cellSize);
            sw.WriteLine("origin: [0.0, 0.0, 0.0]");
            sw.WriteLine("negate: 0");
            sw.WriteLine("occupied_thresh: 0.65");
            sw.WriteLine("free_thresh: 0.196");
        }
    }

    public void SpawnRobots(int numRobots)
    {
        if (robotPrefab == null)
        {
            Debug.LogWarning("RobotSpawner: prefab not found!");
            return;
        }

        if (robotsParent == null)
        {
            GameObject parentObj = new GameObject("SpawnedRobots");
            robotsParent = parentObj.transform;
        }

        for (int i = 0; i < numRobots; i++)
        {
            SpawnSingleRobot(chargingPoints[i]);
        }

        Debug.Log($"RobotSpawner: Spawned {numRobots} robot.");
    }

    private void SpawnSingleRobot(Vector3 position)
    {
        Instantiate(robotPrefab, position, Quaternion.identity, robotsParent);
    }

    /// <summary>
    /// Spawns rocks at random positions with random rotations
    /// </summary>
    public void SpawnRocks()
    {
        if (rockPrefabs == null || rockPrefabs.Length == 0)
        {
            Debug.LogWarning("RockSpawner: No rock prefabs assigned!");
            return;
        }

        // Create parent object if not assigned
        if (rocksParent == null)
        {
            GameObject parentObj = new GameObject("SpawnedRocks");
            rocksParent = parentObj.transform;
        }

        for (int i = 0; i < numberOfRocks; i++)
        {
            SpawnSingleRock();
        }

        //Debug.Log($"RockSpawner: Spawned {numberOfRocks} rocks");
    }

    /// <summary>
    /// Spawns a single rock at a random position
    /// </summary>
    public void SpawnSingleRock()
    {
        // Select random rock prefab
        GameObject selectedPrefab = rockPrefabs[Random.Range(0, rockPrefabs.Length)];

        // Generate random position
        Vector3 randomPosition = new Vector3(
            Random.Range(minPosition.x, maxPosition.x),
            Random.Range(minPosition.y, maxPosition.y),
            Random.Range(minPosition.z, maxPosition.z)
        );

        // Generate random rotation (X and Z vary, Y is fixed)
        Quaternion randomRotation = Quaternion.Euler(
            Random.Range(minRotationX, maxRotationX),
            fixedRotationY,
            Random.Range(minRotationZ, maxRotationZ)
        );

        // Instantiate the rock
        GameObject spawnedRock = Instantiate(selectedPrefab, randomPosition, randomRotation, rocksParent);

        // Apply random scale if enabled
        if (randomizeScale)
        {
            float randomScale = Random.Range(minScale, maxScale);
            spawnedRock.transform.localScale *= randomScale;
        }

        if (spawnedRock.GetComponent<MeshCollider>() == null)
        {
            MeshCollider meshCollider = spawnedRock.AddComponent<MeshCollider>();
            meshCollider.convex = true; // puoi impostarlo su true se vuoi usarlo con Rigidbody
        }
    }

    public void SpawnExcavationPoints(int numRobots)
    {
        if (excPointPrefab == null)
        {
            Debug.LogWarning("Excavation Point Spawner: prefab not found!");
            return;
        }

        if (excPointParent == null)
        {
            GameObject parentObj = new GameObject("SpawnedExcPoint");
            excPointParent = parentObj.transform;
        }

        for (int i = 0; i < numRobots; i++)
        {
            SpawnSingleExcPoint();
        }

        Debug.Log($"ExcPointSpawner: Spawned {numRobots} excavation point.");
    }

    public void SpawnSingleExcPoint()
    {
        // Generate random position
        Vector3 randomPosition = new Vector3(
            Random.Range(minPosition.x, maxPosition.x),
            Random.Range(minPosition.y, maxPosition.y),
            Random.Range(minPosition.z, maxPosition.z)
        );

        // Generate random rotation (X and Z vary, Y is fixed)
        Quaternion rotation = Quaternion.Euler(
            0f,
            0f,
            0f
        );

        // Instantiate the rock
        GameObject spawnedExcPoint = Instantiate(excPointPrefab, randomPosition, rotation, excPointParent);

        if (spawnedExcPoint.GetComponent<MeshCollider>() == null)
        {
            MeshCollider meshCollider = spawnedExcPoint.AddComponent<MeshCollider>();
            meshCollider.convex = true; // puoi impostarlo su true se vuoi usarlo con Rigidbody
        }
    }

    /// <summary>
    /// Clears all spawned rocks
    /// </summary>
    public void ClearRocks()
    {
        if (rocksParent != null)
        {
            // Destroy all children
            foreach (Transform child in rocksParent)
            {
                Destroy(child.gameObject);
            }
           // Debug.Log("RockSpawner: Cleared all rocks");
        }
    }

    /// <summary>
    /// Respawns all rocks (clears and spawns new ones)
    /// </summary>
    public void RespawnRocks()
    {
        ClearRocks();
        SpawnRocks();
    }

    // Editor visualization
    void OnDrawGizmosSelected()
    {
        // Draw spawn bounds
        Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
        Vector3 center = (minPosition + maxPosition) / 2f;
        Vector3 size = maxPosition - minPosition;
        Gizmos.DrawWireCube(center, size);
    }


}
