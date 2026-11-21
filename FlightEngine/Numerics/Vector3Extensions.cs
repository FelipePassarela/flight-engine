using System.Numerics;

namespace FlightEngine.Numerics;

internal static class Vector3Extensions
{
    public static bool AllComponentsAreFinite(this Vector3 vector)
    {
        var finiteComponentMask = Vector3.IsFinite(vector);
        return Vector3.AllWhereAllBitsSet(finiteComponentMask);
    }
}
