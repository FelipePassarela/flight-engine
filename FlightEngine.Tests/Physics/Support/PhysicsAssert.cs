using System.Numerics;

namespace FlightEngine.Physics.Tests.Physics.Support;

internal static class PhysicsAssert
{
    public static void PoseNear(Vector3 expectedPosition, Quaternion expectedOrientation,
        RigidBody body, float positionTolerance = 1e-5f)
    {
        VectorNear(expectedPosition, body.Transform.Position, positionTolerance);
        OrientationNear(expectedOrientation, body.Transform.Orientation);
    }

    public static void AtRest(RigidBody body, float tolerance = 1e-5f)
    {
        VectorNear(Vector3.Zero, body.Velocity, tolerance);
        VectorNear(Vector3.Zero, body.AngularVelocity, tolerance);
    }

    public static void VectorNear(Vector3 expected, Vector3 actual, float tolerance = 1e-5f)
    {
        var error = Vector3.Distance(expected, actual);
        Assert.True(
            float.IsFinite(error) && error <= tolerance,
            $"Expected {expected}, actual {actual}; error {error}, tolerance {tolerance}.");
    }

    public static void OrientationNear(Quaternion expected, Quaternion actual, float toleranceRadians = 1e-3f)
    {
        // q and -q describe the same orientation.
        var dot = Quaternion.Dot(Quaternion.Normalize(expected), Quaternion.Normalize(actual));
        dot = MathF.Abs(dot);
        var angle = 2 * MathF.Acos(Math.Clamp(dot, 0, 1));
        
        Assert.True(
            float.IsFinite(angle) && angle <= toleranceRadians,
            $"Orientation error {angle} radians exceeds {toleranceRadians}.");
        Assert.InRange(MathF.Abs(actual.Length() - 1), 0, 1e-5f);
    }
}
