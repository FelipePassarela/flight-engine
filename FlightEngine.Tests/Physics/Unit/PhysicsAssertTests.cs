using System.Numerics;
using FlightEngine.Physics.Tests.Physics.Support;
using Xunit.Sdk;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class PhysicsAssertTests
{
    [Fact]
    public void OrientationNearRejectsAnAngleJustOverTolerance()
    {
        var actual = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 0.0011f);

        Assert.Throws<TrueException>(() =>
            PhysicsAssert.OrientationNear(Quaternion.Identity, actual, toleranceRadians: 0.001f));
    }

    [Fact]
    public void OrientationNearAcceptsEquivalentQuaternionSigns()
    {
        var orientation = Quaternion.CreateFromYawPitchRoll(0.4f, -0.2f, 0.8f);
        var equivalentOrientation = Quaternion.Negate(orientation);

        PhysicsAssert.OrientationNear(orientation, equivalentOrientation, toleranceRadians: 1e-6f);
    }

    [Fact]
    public void RotatingAroundAxisRejectsPredominantlyPerpendicularMotion()
    {
        var body = PhysicsObjects.Body();
        body.AngularVelocity = new Vector3(1, 0, 0.3f);

        Assert.Throws<TrueException>(() =>
            PhysicsAssert.RotatingAroundAxis(body, Vector3.UnitZ,
                minimumAxialSpeed: 0.25f, maximumOffAxisSpeed: 1e-3f));
    }
}
