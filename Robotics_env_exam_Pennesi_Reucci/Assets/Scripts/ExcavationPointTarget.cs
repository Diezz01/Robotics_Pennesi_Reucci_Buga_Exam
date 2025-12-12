using UnityEngine;

/// <summary>
/// Data wrapper for ExcavationPoint component to use in navigation
/// </summary>
public class ExcavationPointTarget : MapTarget
{
    public ExcavationPoint.ExcavationType Type;

    public ExcavationPointTarget(Vector3 position, ExcavationPoint.ExcavationType type)
    {
        Position = position;
        Type = type;
    }
}
