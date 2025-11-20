using UnityEngine;

public class RockSpawner : MonoBehaviour
{
    [Header("Rock Prefabs")]
    [Tooltip("Array of rock prefabs to spawn")]
    public GameObject[] rockPrefabs;

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

    void Start()
    {
        if (spawnOnStart)
        {
            SpawnRocks();
        }
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

        Debug.Log($"RockSpawner: Spawned {numberOfRocks} rocks");
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
            Debug.Log("RockSpawner: Cleared all rocks");
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
