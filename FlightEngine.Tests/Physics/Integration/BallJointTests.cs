using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class BallJointTests
{
    [Theory]
    [InlineData(1f, 0f, 0f)]
    [InlineData(0f, 1f, 0f)]
    [InlineData(0f, 0f, 1f)]
    public void CenteredPivotAllowsRelativeRotationAroundEachAxis(float x, float y, float z)
    {
        var axis = new Vector3(x, y, z);
        var pivot = new Vector3(1f, 2f, 3f);
        var fixedBody = PhysicsObjects.Body(mass: 0f, position: pivot);
        var movingBody = PhysicsObjects.Body(position: pivot);
        var joint = new BallJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero);
        var simulation = CreateSimulation(joint);

        const int durationSeconds = 1;
        const float angularSpeed = 1.25f;
        var initialAngularVelocity = axis * angularSpeed;
        movingBody.AngularVelocity = initialAngularVelocity;

        simulation.RunFor(durationSeconds, stepsPerSecond: 60);

        const float expectedAngle = angularSpeed * durationSeconds;
        var expectedOrientation = Quaternion.CreateFromAxisAngle(axis, expectedAngle);
        PhysicsAssert.PoseNear(pivot, expectedOrientation, movingBody);
        PhysicsAssert.VectorNear(initialAngularVelocity, movingBody.AngularVelocity);
    }

    [Fact]
    public void SeparatedAnchorsConvergeToTheSamePosition()
    {
        var fixedBody = PhysicsObjects.Body(mass: 0f);
        var initialPosition = new Vector3(1f, -2f, 3f);
        var movingBody = PhysicsObjects.Body(position: initialPosition);
        var joint = new BallJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero);
        var simulation = CreateSimulation(joint);

        var remainingSeparationTolerance = 0.01f * initialPosition.Length();

        simulation.RunFor(durationSeconds: 1, stepsPerSecond: 60);

        PhysicsAssert.VectorNear(Vector3.Zero, movingBody.Transform.Position, remainingSeparationTolerance);
    }

    [Theory]
    [InlineData(60)]
    [InlineData(120)]
    public void PendulumSwingsUnderGravityAndKeepsItsPivotAttached(int stepsPerSecond)
    {
        var pivot = Vector3.Zero;
        var initialPosition = Vector3.Normalize(new Vector3(1f, -1f, 0f));
        var fixedBody = PhysicsObjects.Body(mass: 0f, position: pivot);
        var movingBody = PhysicsObjects.Body(position: initialPosition, gravity: Gravity.EarthGravityAcceleration);
        var localAnchor = movingBody.Transform.InverseTransformPoint(pivot);
        var joint = new BallJoint(fixedBody, movingBody, Vector3.Zero, localAnchor);
        var simulation = CreateSimulation(joint);

        const float pivotTolerance = 0.01f;
        var leftmostPosition = initialPosition.X;
        simulation.RunFor(durationSeconds: 3, stepsPerSecond, afterStep: CheckPivotAndTrackSwing);

        Assert.True(leftmostPosition < -0.25f, $"Expected the pendulum to swing past vertical, reached x={leftmostPosition}.");
        return;

        void CheckPivotAndTrackSwing(float _)
        {
            var movingAnchor = movingBody.Transform.TransformPoint(localAnchor);
            PhysicsAssert.VectorNear(pivot, movingAnchor, pivotTolerance);
            leftmostPosition = MathF.Min(leftmostPosition, movingBody.Transform.Position.X);
        }
    }

    private static PhysicsSimulation CreateSimulation(BallJoint joint) =>
        new(joint.BodyA.GameObject, joint.BodyB.GameObject, new GameObject([joint]));
}
