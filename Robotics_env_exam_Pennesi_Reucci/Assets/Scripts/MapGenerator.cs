using RosMessageTypes.Nav;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using static UnityEditor.PlayerSettings;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class OccupancyGridGenerator : MonoBehaviour
{

    [Header("Rock Prefabs")]
    [Tooltip("Array of rock prefabs to spawn")]
    public GameObject[] rockPrefabs;

    public GameObject excPointPrefab;

    public Transform excPointParent;

    [Header("Spawn Settings")]
    [Tooltip("Number of rocks to spawn")]
    public int numberOfRocks = 30;

    private int robotCounter = 1;

    [Tooltip("Minimum position bounds for spawning")]
    public Vector3 minPosition = new Vector3(-48, 0, -48);

    [Tooltip("Maximum position bounds for spawning")]
    public Vector3 maxPosition = new Vector3(48, 0, 48);

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
    public float maxScale = 1.0f;

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
    public string topicRobotsName = "/robots";
    public string topicTargetsName = "/targets";


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
        PoseArrayMsg msgRobots = new PoseArrayMsg();
        PoseArrayMsg msgTargets = new PoseArrayMsg();
        int numRobots = Random.Range(1, chargingPoints.Length + 1);

        if (spawnOnStart)
        {
            SpawnRobots(numRobots);
            SpawnRocks();
            msgTargets = SpawnExcavationPoints(numRobots);
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OccupancyGridMsg>(topicName);
        ros.RegisterPublisher<PoseArrayMsg>(topicRobotsName);
        ros.RegisterPublisher<PoseArrayMsg>(topicTargetsName);

        int[,] grid = new int[mapWidth, mapHeight];

        Vector3 origin = transform.position - 
                        new Vector3(mapWidth * cellSize, 0, mapHeight * cellSize) * 0.5f;

        Vector3 halfExtents = new Vector3(cellSize / 2f, heightCheck / 2f, cellSize / 2f);
            
        for (int x = 0; x < mapWidth; x++)
        {
            for (int y = 0; y < mapHeight; y++)
            {
                Vector3 cellCenter =
                    origin + new Vector3(x * cellSize + cellSize * 0.5f,
                                        heightCheck * 0.5f,
                                        y * cellSize + cellSize * 0.5f);

                bool blocked = Physics.CheckBox(
                    cellCenter,
                    halfExtents,
                    Quaternion.identity,
                    obstacleLayer
                );

                int gridY = mapHeight - 1 - y;  
                if(blocked){
                    grid[x, gridY] = 1;
                    
                }else{
                    grid[x, gridY] = 0;
                }
            }
        }

        //SavePGM(grid);
        //SaveYAML();

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

        // crea un array di PoseMsg della dimensione corretta
        msgRobots.poses = new PoseMsg[numRobots];

        // riempi il PoseArrayMsg con le prime numRobots coordinate
        for (int i = 0; i < numRobots; i++)
        {
            Vector3 point = chargingPoints[i];
            msgRobots.poses[i] = new PoseMsg(
                new PointMsg(point.x, point.y, point.z),      // POSITION
                new QuaternionMsg(0.0, 0.0, 0.0, 1.0)        // ORIENTATION fissa
            );
        }

        ros.Publish(topicRobotsName, msgRobots);
        ros.Publish(topicTargetsName, msgTargets);

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
        GameObject spawnedRobot = Instantiate(robotPrefab, position, Quaternion.identity, robotsParent);

        ExcavatorController excavatorScript = spawnedRobot.GetComponent<ExcavatorController>();
        SimplePathFollower pathFollowerScript = spawnedRobot.GetComponent<SimplePathFollower>();

        excavatorScript.robotId = $"tb3_{robotCounter}";
        pathFollowerScript.topicName = $"/{excavatorScript.robotId}/astar_path";
        robotCounter ++;
        excavatorScript.Initialize(excavatorScript.robotId);

        var values = System.Enum.GetValues(typeof(ExcavationPoint.ExcavationType));

        excavatorScript.RobotType = (ExcavatorController.ExcavationRobotType)
                            values.GetValue(Random.Range(0, values.Length));

        excavatorScript.chargingStationPosition = position;

        Debug.Log($"<color=green>Generated Robot: {excavatorScript.robotId}  Type: {excavatorScript.RobotType} | Pos: {excavatorScript.chargingStationPosition}</color>");
    
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

    // Area where robots and base are placed
    private bool IsInsideForbiddenArea(Vector3 pos)
    {
        return pos.x >= 10f && pos.x <= 49f &&
               pos.z >= -49f && pos.z <= -35f;
    }

    private Vector3 GetValidRandomPosition()
    {
        Vector3 pos;

        do
        {
            pos = new Vector3(
                Random.Range(minPosition.x, maxPosition.x),
                Random.Range(minPosition.y, maxPosition.y),
                Random.Range(minPosition.z, maxPosition.z)
            );

            // Arrotonda alle coordinate intere
            pos = new Vector3(
                Mathf.Round(pos.x),
                Mathf.Round(pos.y),
                Mathf.Round(pos.z)
            );

        } while (IsInsideForbiddenArea(pos));  // RIGENERA se � dentro l�area proibita

        return pos;
    }
    public void SpawnSingleRock()
    {
        GameObject selectedPrefab = rockPrefabs[Random.Range(0, rockPrefabs.Length)];

        // Usa la funzione che genera SOLO posizioni valide
        Vector3 randomPosition = GetValidRandomPosition();

        Quaternion randomRotation = Quaternion.Euler(
            Random.Range(minRotationX, maxRotationX),
            fixedRotationY,
            Random.Range(minRotationZ, maxRotationZ)
        );

        GameObject spawnedRock = Instantiate(selectedPrefab, randomPosition, randomRotation, rocksParent);

        if (randomizeScale)
        {
            float randomScale = Random.Range(minScale, maxScale);
            spawnedRock.transform.localScale *= randomScale;
        }

        if (spawnedRock.GetComponent<MeshCollider>() == null)
        {
            MeshCollider meshCollider = spawnedRock.AddComponent<MeshCollider>();
            meshCollider.convex = true;
        }
    }


    public PoseArrayMsg SpawnExcavationPoints(int numRobots)
    {
        PoseArrayMsg msgTargets = new PoseArrayMsg();
        msgTargets.poses = new PoseMsg[numRobots];

        if (excPointPrefab == null)
        {
            Debug.LogWarning("Excavation Point Spawner: prefab not found!");
            return null;
        }

        if (excPointParent == null)
        {
            GameObject parentObj = new GameObject("SpawnedExcPoint");
            excPointParent = parentObj.transform;
        }

        for (int i = 0; i < numRobots; i++)
        {
            Vector3 pos = SpawnSingleExcPoint();

            msgTargets.poses[i] = new PoseMsg(
                new PointMsg(pos.x, pos.y, pos.z),      // POSITION
                new QuaternionMsg(0.0, 0.0, 0.0, 1.0)        // ORIENTATION fissa
            );

        }

        Debug.Log($"ExcPointSpawner: Spawned {numRobots} excavation point.");
        return msgTargets;
    }

    public Vector3 SpawnSingleExcPoint()
    {
        Vector3 randomPosition = GetValidRandomPosition();

        Quaternion rotation = Quaternion.identity;

        GameObject spawnedExcPoint = Instantiate(excPointPrefab, randomPosition, rotation, excPointParent);
        spawnedExcPoint.tag = "ExcavationPoint";

        ExcavationPoint excPointScript = spawnedExcPoint.GetComponent<ExcavationPoint>();

        if (excPointScript != null)
        {
            var values = System.Enum.GetValues(typeof(ExcavationPoint.ExcavationType));

            excPointScript.Type = (ExcavationPoint.ExcavationType)
                                values.GetValue(Random.Range(0, values.Length));

            excPointScript.Position = randomPosition;

            Debug.Log($"<color=green>Generated Excavation Point Type: {excPointScript.Type} | Pos: {randomPosition}</color>");
        }

        return randomPosition;
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
