using UnityEngine;

public class ExcavationPoint : MapTarget
{
    public enum ExcavationType
    {
        Excavation,
        Analysys,
        GasAnalysys
    }

    public ExcavationType Type;

}
