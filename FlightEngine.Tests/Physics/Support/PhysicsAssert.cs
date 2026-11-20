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
        var angle = AngleBetweenOrientations(expected, actual);

        Assert.True(
            float.IsFinite(angle) && angle <= toleranceRadians,
            $"Orientation error {angle} radians exceeds {toleranceRadians}.");
        Assert.InRange(MathF.Abs(actual.Length() - 1), 0, 1e-5f);
    }

    public static void RotatedByAtLeast(
        Quaternion initialOrientation, Quaternion actualOrientation, float minimumAngleDegrees)
    {
        var angleRadians = AngleBetweenOrientations(initialOrientation, actualOrientation);
        var angleDegrees = angleRadians * (180f / MathF.PI);

        Assert.True(
            float.IsFinite(angleDegrees) && angleDegrees >= minimumAngleDegrees,
            $"Expected rotation of at least {minimumAngleDegrees} degrees, actual rotation {angleDegrees} degrees.");
        Assert.InRange(MathF.Abs(actualOrientation.Length() - 1), 0, 1e-5f);
    }

    public static void RotatingAroundAxis(
        RigidBody body, Vector3 axis, float minimumAxialSpeed, float maximumOffAxisSpeed)
    {
        var unitAxis = Vector3.Normalize(axis);
        var angularVelocity = body.AngularVelocity;
        var axialSpeed = Vector3.Dot(angularVelocity, unitAxis);
        var offAxisSpeed = (angularVelocity - axialSpeed * unitAxis).Length();

        Assert.True(
            float.IsFinite(axialSpeed) && axialSpeed > minimumAxialSpeed,
            $"Expected angular speed above {minimumAxialSpeed} rad/s along {axis}, actual speed {axialSpeed} rad/s.");
        Assert.True(
            float.IsFinite(offAxisSpeed) && offAxisSpeed <= maximumOffAxisSpeed,
            $"Expected angular speed at most {maximumOffAxisSpeed} rad/s perpendicular to {axis}, "
            + $"actual speed {offAxisSpeed} rad/s.");
    }

    private static float AngleBetweenOrientations(Quaternion first, Quaternion second)
    {
        var inverseFirst = Quaternion.Conjugate(Quaternion.Normalize(first));
        var relativeRotation = inverseFirst * Quaternion.Normalize(second);
        var vectorPart = new Vector3(relativeRotation.X, relativeRotation.Y, relativeRotation.Z);
        // The absolute scalar part selects the shortest angle for either quaternion sign.
        return 2 * MathF.Atan2(vectorPart.Length(), MathF.Abs(relativeRotation.W));
    }
}
