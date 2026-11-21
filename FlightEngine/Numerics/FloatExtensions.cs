namespace FlightEngine.Numerics;

internal static class FloatExtensions
{
    public static bool IsPositiveFinite(this float value) => value > 0.0f && float.IsFinite(value);
}
