using UnityEngine;

public class ExcavationPoint : MonoBehaviour
{
    public enum ExcavationType
    {
        Excavation,
        Analysys,
        GasAnalysys
    }

    public ExcavationType Type;
    public Vector3 Position;
}
