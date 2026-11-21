using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class HingeJointTests
{
    [Fact]
    public void CenteredJointAllowsContinuousRotationThroughMultipleTurns()
    {
        var axis = Vector3.Normalize(new Vector3(1f, 2f, 3f));
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var initialOrientation = Quaternion.CreateFromAxisAngle(axis, 0.8f);
        var movingBody = PhysicsObjects.Body(orientation: initialOrientation);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, axis, axis);
        var simulation = CreateSimulation(joint);
        const float angularSpeed = 5f;
        var expectedAngularVelocity = angularSpeed * axis;
        movingBody.AngularVelocity = expectedAngularVelocity;

        simulation.RunFor(durationSeconds: 3, stepsPerSecond: 60, afterStep: AssertFreeRotation);
        return;

        void AssertFreeRotation(float elapsedTime)
        {
            var angle = angularSpeed * elapsedTime;
            var rotation = Quaternion.CreateFromAxisAngle(axis, angle);
            var expectedOrientation = rotation * initialOrientation;
            PhysicsAssert.PoseNear(Vector3.Zero, expectedOrientation, movingBody);
            PhysicsAssert.VectorNear(expectedAngularVelocity, movingBody.AngularVelocity);
        }
    }

    [Theory]
    [InlineData(40f, 1f)]
    [InlineData(0.01f, -1f)]
    [InlineData(0f, -1f)]
    public void MisalignedAxesConvergeToTheSameDirection(float tiltDegrees, float axisSign)
    {
        var hingeAxis = Vector3.UnitZ;
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var tiltRadians = tiltDegrees * (MathF.PI / 180f);
        var orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, tiltRadians);
        var movingBody = PhysicsObjects.Body(orientation: orientation);
        var movingAxis = axisSign * hingeAxis;
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, hingeAxis, movingAxis);
        var simulation = CreateSimulation(joint);

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);

        var alignedAxis = movingBody.Transform.TransformDirection(movingAxis);
        PhysicsAssert.VectorNear(hingeAxis, alignedAxis, tolerance: 1e-4f);
        PhysicsAssert.VectorNear(Vector3.Zero, movingBody.AngularVelocity, tolerance: 1e-4f);
    }

    [Fact]
    public void SeparatedAnchorsConvergeToTheSamePosition()
    {
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var movingBody = PhysicsObjects.Body(position: new Vector3(1f, -2f, 3f));
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, Vector3.UnitZ, Vector3.UnitZ);
        var simulation = CreateSimulation(joint);

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);

        PhysicsAssert.VectorNear(Vector3.Zero, movingBody.Transform.Position, tolerance: 1e-4f);
    }

    [Theory]
    [InlineData(60)]
    [InlineData(120)]
    public void PendulumSwingsInTheHingePlaneAndKeepsItsPivotAttached(int stepsPerSecond)
    {
        var hingeAxis = Vector3.UnitZ;
        var pivot = Vector3.Zero;
        var initialPosition = new Vector3(0.6f, -0.8f, 0f);
        var fixedBody = PhysicsObjects.Body(mass: 0f, position: pivot);
        var movingBody = PhysicsObjects.Body(position: initialPosition, gravity: Gravity.EarthGravityAcceleration);
        var movingAnchor = movingBody.Transform.InverseTransformPoint(pivot);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, movingAnchor, hingeAxis, hingeAxis);
        var simulation = CreateSimulation(joint);
        const float outOfPlaneSpeed = 1f;
        movingBody.Velocity = outOfPlaneSpeed * hingeAxis;
        var leftmostPosition = initialPosition.X;

        simulation.RunFor(durationSeconds: 3, stepsPerSecond, afterStep: CheckPivotAndTrackSwing);

        Assert.True(leftmostPosition < -0.25f, $"Expected the pendulum to swing past vertical, reached x={leftmostPosition}.");
        return;

        void CheckPivotAndTrackSwing(float _)
        {
            var currentPosition = movingBody.Transform.Position;
            var anchorPosition = movingBody.Transform.TransformPoint(movingAnchor);
            const float pivotTolerance = 0.01f;
            const float planeTolerance = 1e-4f;
            PhysicsAssert.VectorNear(pivot, anchorPosition, pivotTolerance);
            Assert.InRange(currentPosition.Z, -planeTolerance, planeTolerance);
            leftmostPosition = MathF.Min(leftmostPosition, currentPosition.X);
        }
    }

    private static PhysicsSimulation CreateSimulation(HingeJoint joint) =>
        new(joint.BodyA.GameObject, joint.BodyB.GameObject, new GameObject([joint]));
}
