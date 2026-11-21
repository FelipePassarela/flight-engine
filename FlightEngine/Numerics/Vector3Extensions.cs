using System.Numerics;

namespace FlightEngine.Numerics;

internal static class Vector3Extensions
{
    public static bool HasFiniteComponents(this Vector3 vector)
    {
        var finiteComponents = Vector3.IsFinite(vector);
        return Vector3.AllWhereAllBitsSet(finiteComponents);
    }
}
