using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class HingeLimitTests
{
    private const float DeltaTime = 1f / 60f;

    [Theory]
    [InlineData(float.NaN, 0.4f, "minimumRadians")]
    [InlineData(-0.3f, float.PositiveInfinity, "maximumRadians")]
    [InlineData(-MathF.PI, 0.4f, "minimumRadians")]
    [InlineData(-0.3f, MathF.PI, "maximumRadians")]
    [InlineData(0.4f, -0.3f, "maximumRadians")]
    [InlineData(0f, 0f, "maximumRadians")]
    public void LimitsRequireAnOrderedFiniteIntervalWithinOneTurn(
        float minimumRadians, float maximumRadians, string parameterName)
    {
        Assert.Throws<ArgumentOutOfRangeException>(parameterName,
            () => new HingeAngleLimits(minimumRadians, maximumRadians));
    }

    [Fact]
    public void AxialLimitAndOffCenterAnchorAreSolvedTogether()
    {
        var axis = Vector3.UnitX;
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body(inertia: 0.5f, position: -Vector3.UnitZ);
        var movingAnchor = Vector3.UnitZ;
        const float maximumAngle = 0.4f;
        var limits = new HingeAngleLimits(-0.3f, maximumAngle);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, movingAnchor, axis, axis, limits);

        movingBody.AngularVelocity = 60f * axis;
        var anchorOffset = movingBody.Transform.TransformVector(movingAnchor);
        movingBody.Velocity = -Vector3.Cross(movingBody.AngularVelocity, anchorOffset);

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var expectedAngularVelocity = (maximumAngle / DeltaTime) * axis;
        PhysicsAssert.VectorNear(expectedAngularVelocity, movingBody.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.Zero, movingBody.GetVelocityAtPoint(movingAnchor));
    }

    [Fact]
    public void LaterSolverIterationReleasesTheAccumulatedStopImpulse()
    {
        var axis = Vector3.UnitX;
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body(inertia: 1f);
        const float maximumAngle = 0.4f;
        var limits = new HingeAngleLimits(-0.3f, maximumAngle);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, axis, axis, limits);
        const float outwardSpeed = 60f;
        movingBody.AngularVelocity = outwardSpeed * axis;
        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);
        PhysicsAssert.VectorNear((maximumAngle / DeltaTime) * axis, movingBody.AngularVelocity);

        // Another constraint reverses the body's free motion during the same step.
        const float inwardSpeed = 2f;
        movingBody.ApplyAngularImpulse(-(outwardSpeed + inwardSpeed) * axis);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(-inwardSpeed * axis, movingBody.AngularVelocity);
    }

    [Fact]
    public void ZeroAngularResponseAtTheLimitStillAllowsAnchorCorrection()
    {
        var axis = Vector3.UnitX;
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body(inertia: 0f);
        var limits = new HingeAngleLimits(-0.3f, 0.4f);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, axis, axis, limits);
        joint.Prepare(DeltaTime);

        movingBody.Transform.Orientation = Quaternion.CreateFromAxisAngle(axis, 0.7f);
        movingBody.Velocity = Vector3.One;
        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.AtRest(movingBody);
    }
}
