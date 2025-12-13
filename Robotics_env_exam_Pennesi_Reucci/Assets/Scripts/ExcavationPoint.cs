using UnityEngine;

public class ExcavationPoint : MapTarget
{
    public enum ExcavationType
    {
        Excavation = 0,
        Analysys = 1,
        GasAnalysys = 2
    }

    public ExcavationType Type;

}
