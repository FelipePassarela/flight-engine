using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class DistanceJointTests
{
    private const float DeltaTime = 1f / 60f;

    [Theory]
    [InlineData(0f)]
    [InlineData(-1f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(float.NegativeInfinity)]
    public void ConstructorRejectsInvalidTargetDistance(float targetDistance)
    {
        var bodyA = PhysicsObjects.Body();
        var bodyB = PhysicsObjects.Body(position: Vector3.UnitX);

        Assert.Throws<ArgumentOutOfRangeException>(nameof(targetDistance), () =>
        {
            _ = new DistanceJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, targetDistance);
        });
    }

    [Theory]
    [InlineData(-1f)]
    [InlineData(1f)]
    public void AtTargetDistanceRemovesOnlyTheRadialVelocity(float radialSpeed)
    {
        const float targetDistance = 1f;
        var joint = CreateJoint(targetDistance, initialDistance: targetDistance);
        var movingBody = joint.BodyB;
        var tangentialVelocity = Vector3.UnitY;
        movingBody.Velocity = radialSpeed * Vector3.UnitX + tangentialVelocity;

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(tangentialVelocity, movingBody.Velocity);
    }

    [Theory]
    [InlineData(0.5f)]
    [InlineData(2f)]
    public void OutsideTargetDistanceVelocityCorrectionReducesDistanceError(float initialDistance)
    {
        const float targetDistance = 1.5f;
        var joint = CreateJoint(targetDistance, initialDistance);

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var radialDisplacement = joint.BodyB.Velocity.X * DeltaTime;
        var projectedDistance = MathF.Abs(initialDistance + radialDisplacement);
        var initialError = MathF.Abs(initialDistance - targetDistance);
        var projectedError = MathF.Abs(projectedDistance - targetDistance);
        Assert.True(projectedError < initialError,
            $"Expected the distance error to decrease from {initialError}, but it became {projectedError}.");
    }

    [Theory]
    [InlineData(1f, 0.5f)]
    [InlineData(3f, 0.25f)]
    public void OffAxisAnchorSharesCorrectionBetweenTranslationAndRotation(float inertia, float expectedAngularSpeed)
    {
        const float targetDistance = 1f;
        var localAnchor = Vector3.UnitY;
        var fixedBody = PhysicsObjects.Body(mass: 0);
        var movingPosition = Vector3.UnitX * targetDistance - localAnchor;
        var movingBody = PhysicsObjects.Body(inertia: inertia, position: movingPosition);
        movingBody.Velocity = Vector3.UnitX;
        var joint = new DistanceJoint(fixedBody, movingBody, Vector3.Zero, localAnchor, targetDistance);
        joint.OnInitialize();

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var radialSpeed = movingBody.GetVelocityAtPoint(localAnchor).X;
        Assert.Equal(0f, radialSpeed, precision: 5);
        PhysicsAssert.VectorNear(expectedAngularSpeed * Vector3.UnitZ, movingBody.AngularVelocity);
    }

    [Fact]
    public void OffAxisAnchorsAccountForBothBodiesAngularInertia()
    {
        const float targetDistance = 1f;
        var localAnchorA = Vector3.UnitY;
        var localAnchorB = -2f * Vector3.UnitY;
        var positionB = Vector3.UnitX * targetDistance - localAnchorB;
        var bodyA = PhysicsObjects.Body(inertia: 1f, position: -localAnchorA);
        var bodyB = PhysicsObjects.Body(inertia: 4f, position: positionB);
        // The two inverse masses and the two angular contributions each add 1 to the inverse effective mass.
        bodyB.Velocity = 4f * Vector3.UnitX;
        var joint = new DistanceJoint(bodyA, bodyB, localAnchorA, localAnchorB, targetDistance);
        joint.OnInitialize();

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var radialSpeedA = bodyA.GetVelocityAtPoint(localAnchorA).X;
        var radialSpeedB = bodyB.GetVelocityAtPoint(localAnchorB).X;
        Assert.Equal(radialSpeedA, radialSpeedB, precision: 5);
        PhysicsAssert.VectorNear(-Vector3.UnitZ, bodyA.AngularVelocity);
        PhysicsAssert.VectorNear(-0.5f * Vector3.UnitZ, bodyB.AngularVelocity);
    }

    [Fact]
    public void CoincidentAnchorsKeepMotionFinite()
    {
        var joint = CreateJoint(targetDistance: 1f, initialDistance: 0f);

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.AtRest(joint.BodyA);
        var movingBody = joint.BodyB;
        Assert.True(float.IsFinite(movingBody.Velocity.LengthSquared()),
            $"Expected finite linear velocity, got {movingBody.Velocity}.");
        Assert.True(float.IsFinite(movingBody.AngularVelocity.LengthSquared()),
            $"Expected finite angular velocity, got {movingBody.AngularVelocity}.");
    }

    [Fact]
    public void CoincidentAnchorsStartSeparatingTowardPositiveTargetDistance()
    {
        const float targetDistance = 1f;
        var joint = CreateJoint(targetDistance, initialDistance: 0f);

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var fixedAnchorVelocity = joint.BodyA.GetVelocityAtPoint(joint.LocalAnchorA);
        var movingAnchorVelocity = joint.BodyB.GetVelocityAtPoint(joint.LocalAnchorB);
        var separationSpeed = (movingAnchorVelocity - fixedAnchorVelocity).Length();
        Assert.True(float.IsFinite(separationSpeed) && separationSpeed > 0f,
            $"Expected coincident anchors to start separating at a finite speed, got {separationSpeed}.");
    }

    [Fact]
    public void TwoStaticBodiesStayAtRestDespiteDistanceError()
    {
        const float targetDistance = 0.5f;
        var bodyA = PhysicsObjects.Body(mass: 0);
        var bodyB = PhysicsObjects.Body(mass: 0, position: Vector3.UnitX);
        var joint = new DistanceJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, targetDistance);
        joint.OnInitialize();

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.AtRest(bodyA);
        PhysicsAssert.AtRest(bodyB);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(-1f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(float.NegativeInfinity)]
    public void InvalidTimeStepLeavesVelocitiesUnchanged(float deltaTime)
    {
        var joint = CreateJoint(targetDistance: 1f, initialDistance: 2f);
        var movingBody = joint.BodyB;
        var initialVelocity = Vector3.UnitX;
        var initialAngularVelocity = Vector3.UnitZ;
        movingBody.Velocity = initialVelocity;
        movingBody.AngularVelocity = initialAngularVelocity;

        joint.Prepare(deltaTime);
        joint.SolveVelocity(deltaTime);

        PhysicsAssert.AtRest(joint.BodyA);
        PhysicsAssert.VectorNear(initialVelocity, movingBody.Velocity);
        PhysicsAssert.VectorNear(initialAngularVelocity, movingBody.AngularVelocity);
    }

    [Fact]
    public void InvalidPreparationDiscardsThePreviousStep()
    {
        const float targetDistance = 1f;
        var joint = CreateJoint(targetDistance, initialDistance: targetDistance);
        var movingBody = joint.BodyB;
        var initialVelocity = Vector3.UnitX;
        movingBody.Velocity = initialVelocity;
        joint.Prepare(DeltaTime);

        joint.Prepare(0f);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(initialVelocity, movingBody.Velocity);
    }

    [Theory]
    [InlineData(8f, 0f)]
    [InlineData(12f, 24f)]
    [InlineData(20f, -24f)]
    [InlineData(24f, 0f)]
    public void AppliesLargeImpulsesOnlyWhenRepresentable(float initialDistance, float expectedRadialSpeed)
    {
        const float targetDistance = 16f;
        // At this mass, a 4 m error requires about 2.4e38, while an 8 m error exceeds float.MaxValue.
        const float movingMass = 1e37f;
        var joint = CreateJoint(targetDistance, initialDistance, movingMass);
        var movingBody = joint.BodyB;
        var initialTangentialVelocity = Vector3.UnitY;
        var initialAngularVelocity = Vector3.UnitZ;
        movingBody.Velocity = initialTangentialVelocity;
        movingBody.AngularVelocity = initialAngularVelocity;

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var expectedVelocity = expectedRadialSpeed * Vector3.UnitX + initialTangentialVelocity;
        PhysicsAssert.VectorNear(expectedVelocity, movingBody.Velocity, tolerance: 1e-4f);
        PhysicsAssert.VectorNear(initialAngularVelocity, movingBody.AngularVelocity);
        PhysicsAssert.AtRest(joint.BodyA);
    }

    private static DistanceJoint CreateJoint(float targetDistance, float initialDistance, float movingMass = 1f)
    {
        var fixedLocalAnchor = Vector3.UnitX;
        var movingLocalAnchor = -Vector3.UnitX;
        var anchorSeparation = Vector3.UnitX * initialDistance;
        var movingPosition = fixedLocalAnchor + anchorSeparation - movingLocalAnchor;
        var fixedBody = PhysicsObjects.Body(mass: 0);
        var movingBody = PhysicsObjects.Body(mass: movingMass, position: movingPosition);
        var joint = new DistanceJoint(fixedBody, movingBody, fixedLocalAnchor, movingLocalAnchor, targetDistance, 0.1f);
        joint.OnInitialize();
        return joint;
    }
}