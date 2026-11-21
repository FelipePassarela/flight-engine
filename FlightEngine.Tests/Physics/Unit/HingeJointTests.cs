using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class HingeJointTests
{
    private const float DeltaTime = 1f / 60f;

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CenteredJointBlocksTransverseRotationAndPreservesAxialRotation(bool staticBodyIsA)
    {
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body();
        var bodyA = staticBodyIsA ? fixedBody : movingBody;
        var bodyB = staticBodyIsA ? movingBody : fixedBody;
        var joint = new HingeJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, Vector3.UnitZ, Vector3.UnitZ);
        movingBody.AngularVelocity = new Vector3(2f, -3f, 4f);
        var expectedAngularVelocity = 4f * Vector3.UnitZ;

        SolveOnce(joint);

        PhysicsAssert.VectorNear(expectedAngularVelocity, movingBody.AngularVelocity);
        PhysicsAssert.AtRest(fixedBody);
    }

    [Fact]
    public void OffCenterJointMatchesAnchorVelocitiesAndConservesMomentum()
    {
        var orientationA = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.6f);
        var orientationB = Quaternion.CreateFromAxisAngle(Vector3.UnitY, -0.4f);
        var bodyA = PhysicsObjects.Body(mass: 2f, inertia: 3f, position: -Vector3.UnitX, orientation: orientationA);
        var bodyB = PhysicsObjects.Body(mass: 5f, inertia: 4f, position: new Vector3(2f, 1f, 0f), orientation: orientationB);
        bodyA.Transform.Scale = new Vector3(2f, 0.5f, 1f);
        bodyB.Transform.Scale = new Vector3(0.5f, 1f, 2f);

        var pivot = new Vector3(0f, 2f, -1f);
        var hingeAxis = Vector3.Normalize(new Vector3(1f, 2f, 3f));
        var joint = CreateJointAtWorldPivot(bodyA, bodyB, pivot, hingeAxis);
        bodyA.Velocity = new Vector3(2f, 3f, -1f);
        bodyB.Velocity = new Vector3(-1f, 1f, 2f);
        bodyA.AngularVelocity = new Vector3(0.5f, 2f, -0.25f);
        bodyB.AngularVelocity = new Vector3(-1f, 0.125f, 1f);
        var initialLinearMomentum = PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB);
        var initialAngularMomentum = PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB);

        SolveOnce(joint);

        var anchorVelocityA = bodyA.GetVelocityAtPoint(joint.LocalAnchorA);
        var anchorVelocityB = bodyB.GetVelocityAtPoint(joint.LocalAnchorB);
        var relativeAngularVelocity = bodyB.AngularVelocity - bodyA.AngularVelocity;
        var relativeAxialSpeed = Vector3.Dot(relativeAngularVelocity, hingeAxis);
        PhysicsAssert.VectorNear(anchorVelocityA, anchorVelocityB);
        PhysicsAssert.VectorNear(relativeAxialSpeed * hingeAxis, relativeAngularVelocity);
        PhysicsAssert.VectorNear(initialLinearMomentum, PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB));
        PhysicsAssert.VectorNear(initialAngularMomentum, PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB));
    }

    [Fact]
    public void ZeroInertiaBodiesStillMatchAnchorVelocities()
    {
        var bodyA = PhysicsObjects.Body(inertia: 0f, position: -Vector3.UnitX);
        var bodyB = PhysicsObjects.Body(inertia: 0f, position: Vector3.UnitX);
        var joint = new HingeJoint(bodyA, bodyB, Vector3.UnitX, -Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);
        bodyB.Velocity = new Vector3(2f, -4f, 6f);
        var expectedSharedVelocity = (bodyA.Velocity + bodyB.Velocity) / 2f;

        SolveOnce(joint);

        PhysicsAssert.VectorNear(expectedSharedVelocity, bodyA.Velocity);
        PhysicsAssert.VectorNear(expectedSharedVelocity, bodyB.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyA.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyB.AngularVelocity);
    }

    [Fact]
    public void TwoStaticBodiesStayAtRestWithSeparatedAnchorsAndMisalignedAxes()
    {
        var bodyA = PhysicsObjects.Body(mass: 0f);
        var bodyB = PhysicsObjects.Body(mass: 0f, position: Vector3.UnitX);
        var joint = new HingeJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, Vector3.UnitY, Vector3.UnitZ);

        SolveOnce(joint);

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
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body();
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, Vector3.UnitZ, Vector3.UnitZ);
        var initialAngularVelocity = new Vector3(2f, -3f, 4f);
        movingBody.AngularVelocity = initialAngularVelocity;
        joint.Prepare(DeltaTime);

        joint.Prepare(invalidDeltaTime);
        joint.SolveVelocity(DeltaTime);

        PhysicsAssert.VectorNear(initialAngularVelocity, movingBody.AngularVelocity);
    }

    [Fact]
    public void ConstructorRejectsConnectingABodyToItself()
    {
        var body = PhysicsObjects.Body();

        Assert.Throws<ArgumentException>("bodyB",
            () => new HingeJoint(body, body, Vector3.Zero, Vector3.Zero, Vector3.UnitZ, Vector3.UnitZ));
    }

    [Theory]
    [InlineData(true, float.NaN)]
    [InlineData(false, float.PositiveInfinity)]
    public void ConstructorRejectsNonFiniteAnchors(bool invalidAnchorIsA, float value)
    {
        var bodyA = PhysicsObjects.Body();
        var bodyB = PhysicsObjects.Body();
        var invalidAnchor = new Vector3(0f, value, 0f);
        var anchorA = invalidAnchorIsA ? invalidAnchor : Vector3.Zero;
        var anchorB = invalidAnchorIsA ? Vector3.Zero : invalidAnchor;
        var parameterName = invalidAnchorIsA ? "localAnchorA" : "localAnchorB";

        Assert.Throws<ArgumentOutOfRangeException>(parameterName,
            () => new HingeJoint(bodyA, bodyB, anchorA, anchorB, Vector3.UnitZ, Vector3.UnitZ));
    }

    [Theory]
    [InlineData(true, 0f)]
    [InlineData(false, 0f)]
    [InlineData(true, float.NaN)]
    [InlineData(false, float.PositiveInfinity)]
    public void ConstructorRejectsAxesWithoutAFiniteDirection(bool invalidAxisIsA, float value)
    {
        var bodyA = PhysicsObjects.Body();
        var bodyB = PhysicsObjects.Body();
        var invalidAxis = new Vector3(0f, value, 0f);
        var axisA = invalidAxisIsA ? invalidAxis : Vector3.UnitZ;
        var axisB = invalidAxisIsA ? Vector3.UnitZ : invalidAxis;
        var parameterName = invalidAxisIsA ? "localAxisA" : "localAxisB";

        Assert.Throws<ArgumentOutOfRangeException>(parameterName,
            () => new HingeJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, axisA, axisB));
    }

    [Theory]
    [InlineData(1e-30f)]
    [InlineData(1e30f)]
    public void ConstructorNormalizesFiniteNonzeroAxes(float magnitude)
    {
        var bodyA = PhysicsObjects.Body();
        var bodyB = PhysicsObjects.Body();
        var axisA = magnitude * new Vector3(1f, 2f, 3f);
        var axisB = magnitude * new Vector3(-3f, 2f, -1f);
        var joint = new HingeJoint(bodyA, bodyB, Vector3.Zero, Vector3.Zero, axisA, axisB);

        var expectedAxisA = Vector3.Normalize(new Vector3(1f, 2f, 3f));
        var expectedAxisB = Vector3.Normalize(new Vector3(-3f, 2f, -1f));
        PhysicsAssert.VectorNear(expectedAxisA, joint.LocalAxisA);
        PhysicsAssert.VectorNear(expectedAxisB, joint.LocalAxisB);
    }

    private static HingeJoint CreateJointAtWorldPivot(
        RigidBody bodyA, RigidBody bodyB, Vector3 pivot, Vector3 hingeAxis)
    {
        var localAnchorA = bodyA.Transform.InverseTransformPoint(pivot);
        var localAnchorB = bodyB.Transform.InverseTransformPoint(pivot);
        var localAxisA = bodyA.Transform.InverseTransformDirection(hingeAxis);
        var localAxisB = bodyB.Transform.InverseTransformDirection(hingeAxis);
        return new HingeJoint(bodyA, bodyB, localAnchorA, localAnchorB, localAxisA, localAxisB);
    }

    private static void SolveOnce(HingeJoint joint)
    {
        joint.Prepare(DeltaTime);
        joint.SolveVelocity(DeltaTime);
    }
}
