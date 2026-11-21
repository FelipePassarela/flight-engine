using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class BallJointTests
{
    private const float DeltaTime = 1f / 60f;

    [Fact]
    public void CoincidentOffCenterAnchorsMatchVelocitiesAndConserveMomentum()
    {
        var bodyA = PhysicsObjects.Body(mass: 2f, inertia: 3f, position: -Vector3.UnitX);
        var bodyB = PhysicsObjects.Body(mass: 5f, inertia: 4f, position: new Vector3(2f, 1f, 0f));
        bodyA.Transform.Scale = new Vector3(2f, 0.5f, 1f);
        bodyB.Transform.Scale = new Vector3(0.5f, 1f, 2f);
        var pivot = new Vector3(0f, 2f, -1f);
        var localAnchorA = bodyA.Transform.InverseTransformPoint(pivot);
        var localAnchorB = bodyB.Transform.InverseTransformPoint(pivot);
        var joint = new BallJoint(bodyA, bodyB, localAnchorA, localAnchorB);

        bodyA.Velocity = new Vector3(2f, 3f, -1f);
        bodyB.Velocity = new Vector3(-1f, 1f, 2f);
        bodyA.AngularVelocity = new Vector3(0.5f, 2f, -0.25f);
        bodyB.AngularVelocity = new Vector3(-1f, 0.125f, 1f);
        var initialLinearMomentum = PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB);
        var initialAngularMomentum = PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB);

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        var anchorVelocityA = bodyA.GetVelocityAtPoint(localAnchorA);
        var anchorVelocityB = bodyB.GetVelocityAtPoint(localAnchorB);
        PhysicsAssert.VectorNear(anchorVelocityA, anchorVelocityB);
        PhysicsAssert.VectorNear(initialLinearMomentum, PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB));
        PhysicsAssert.VectorNear(initialAngularMomentum, PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB));
    }

    [Fact]
    public void BodiesWithZeroInertiaStillMatchLinearVelocities()
    {
        var bodyA = PhysicsObjects.Body(inertia: 0f, position: -Vector3.UnitX);
        var bodyB = PhysicsObjects.Body(inertia: 0f, position: Vector3.UnitX);
        var joint = new BallJoint(bodyA, bodyB, Vector3.UnitX, -Vector3.UnitX);
        bodyB.Velocity = new Vector3(2f, -4f, 6f);
        var expectedSharedVelocity = (bodyA.Velocity + bodyB.Velocity) / 2f;

        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(expectedSharedVelocity, bodyA.Velocity);
        PhysicsAssert.VectorNear(expectedSharedVelocity, bodyB.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyA.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyB.AngularVelocity);
    }

    [Fact]
    public void TwoStaticBodiesStayAtRestWithSeparatedAnchors()
    {
        var bodyA = PhysicsObjects.Body(mass: 0f);
        var bodyB = PhysicsObjects.Body(mass: 0f, position: Vector3.UnitY);
        var joint = new BallJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero);

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
    public void InvalidPreparationDiscardsThePreviousStep(float invalidDeltaTime)
    {
        var bodyA = PhysicsObjects.Body(mass: 0f);
        var bodyB = PhysicsObjects.Body();
        var joint = new BallJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero);
        var initialVelocity = new Vector3(1f, 2f, 3f);
        bodyB.Velocity = initialVelocity;
        joint.Prepare(DeltaTime);

        joint.Prepare(invalidDeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(initialVelocity, bodyB.Velocity);
    }

    [Fact]
    public void ConstructorRejectsConnectingABodyToItself()
    {
        var body = PhysicsObjects.Body();

        Assert.Throws<ArgumentException>("bodyB", () => new BallJoint(body, body, Vector3.Zero, Vector3.Zero));
    }

    [Theory]
    [InlineData(true, float.NaN)]
    [InlineData(false, float.PositiveInfinity)]
    public void ConstructorRejectsAnAnchorWithANonFiniteComponent(bool invalidAnchorIsA, float value)
    {
        var bodyA = PhysicsObjects.Body();
        var bodyB = PhysicsObjects.Body();
        var invalidAnchor = new Vector3(0f, value, 0f);
        var localAnchorA = invalidAnchorIsA ? invalidAnchor : Vector3.Zero;
        var localAnchorB = invalidAnchorIsA ? Vector3.Zero : invalidAnchor;
        var invalidParameter = invalidAnchorIsA ? "localAnchorA" : "localAnchorB";

        Assert.Throws<ArgumentOutOfRangeException>(
            invalidParameter, () => new BallJoint(bodyA, bodyB, localAnchorA, localAnchorB));
    }
}
