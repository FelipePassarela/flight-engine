using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class FixedJointTests
{
    [Fact]
    public void JointImpulseConservesLinearAndAngularMomentumAndMatchesAnchorVelocities()
    {
        var bodyA = PhysicsObjects.Body(mass: 2, inertia: 3, position: new Vector3(-1, 0, 0));
        var bodyB = PhysicsObjects.Body(mass: 5, inertia: 4, position: new Vector3(2, 1, 0));
        var anchor = new Vector3(0.3f, 2, -1);
        var localAnchorA = bodyA.Transform.InverseTransformPoint(anchor);
        var localAnchorB = bodyB.Transform.InverseTransformPoint(anchor);
        var jointObject = PhysicsObjects.Joint(bodyA, bodyB, anchor);
        var joint = jointObject.GetComponent<FixedJoint>()!;
        joint.OnInitialize();
        bodyA.Velocity = new Vector3(2, 3, -1);
        bodyB.Velocity = new Vector3(-1, 1, 2);
        bodyA.AngularVelocity = new Vector3(0.5f, 2, -0.2f);
        bodyB.AngularVelocity = new Vector3(-1, 0.1f, 1);
        var momentum = PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB);
        var angularMomentum = PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB);

        joint.Prepare(1f / 60);
        joint.SolveVelocity(1f / 60);

        PhysicsAssert.VectorNear(momentum, PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB));
        PhysicsAssert.VectorNear(angularMomentum, PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB));
        PhysicsAssert.VectorNear(bodyA.GetVelocityAtPoint(localAnchorA), bodyB.GetVelocityAtPoint(localAnchorB));
        PhysicsAssert.VectorNear(bodyA.AngularVelocity, bodyB.AngularVelocity);
    }

    [Theory]
    [InlineData(1f)]
    [InlineData(2f)]
    [InlineData(0.5f)]
    public void SolvingJointPreservesRigidRotationForScaledBodies(float scale)
    {
        var bodyA = PhysicsObjects.Body(position: -Vector3.UnitX);
        var bodyB = PhysicsObjects.Body(position: Vector3.UnitX);
        bodyA.Transform.Scale = bodyB.Transform.Scale = new Vector3(scale);
        var anchor = Vector3.Zero;
        var localAnchorA = bodyA.Transform.InverseTransformPoint(anchor);
        var localAnchorB = bodyB.Transform.InverseTransformPoint(anchor);
        var joint = new FixedJoint(bodyA, bodyB, anchor);
        joint.OnInitialize();
        bodyA.Velocity = -Vector3.UnitY;
        bodyB.Velocity = Vector3.UnitY;
        bodyA.AngularVelocity = bodyB.AngularVelocity = Vector3.UnitZ;
        var angularMomentum = PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB);

        joint.Prepare(1f / 60);
        joint.SolveVelocity(1f / 60);

        var offsetA = bodyA.Transform.TransformPoint(localAnchorA) - bodyA.Transform.TransformPoint(bodyA.CenterOfMass);
        var offsetB = bodyB.Transform.TransformPoint(localAnchorB) - bodyB.Transform.TransformPoint(bodyB.CenterOfMass);
        var anchorVelocityA = bodyA.Velocity + Vector3.Cross(bodyA.AngularVelocity, offsetA);
        var anchorVelocityB = bodyB.Velocity + Vector3.Cross(bodyB.AngularVelocity, offsetB);
        PhysicsAssert.VectorNear(Vector3.Zero, anchorVelocityA);
        PhysicsAssert.VectorNear(Vector3.Zero, anchorVelocityB);
        PhysicsAssert.VectorNear(-Vector3.UnitY, bodyA.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitY, bodyB.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitZ, bodyA.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.UnitZ, bodyB.AngularVelocity);
        PhysicsAssert.VectorNear(angularMomentum, PhysicsMeasurements.CalculateAngularMomentum(bodyA, bodyB));
    }

    [Theory]
    [InlineData(true, 60)]
    [InlineData(false, 60)]
    [InlineData(true, 120)]
    [InlineData(false, 120)]
    public void StaticAttachmentResistsGravityAndOffCenterThrustRegardlessOfBodyOrder(
        bool staticIsBodyA, int stepsPerSecond)
    {
        var fixedBody = PhysicsObjects.Body(mass: 0, position: new Vector3(-2, 1, 3));
        var orientation = Quaternion.CreateFromYawPitchRoll(0.3f, -0.4f, 0.2f);
        var position = new Vector3(1, 2, 3);
        var movingBody = PhysicsObjects.Body(
            mass: 2,
            inertia: 3,
            position: position,
            orientation: orientation,
            gravity: Gravity.EarthGravityAcceleration);
        var joint = staticIsBodyA
            ? PhysicsObjects.Joint(fixedBody, movingBody, new Vector3(2, 4, -1))
            : PhysicsObjects.Joint(movingBody, fixedBody, new Vector3(2, 4, -1));

        var simulation = new PhysicsSimulation(fixedBody.GameObject, movingBody.GameObject, joint);
        var applier = movingBody.GameObject.GetComponent<ForceApplier>()!;
        var thrust = new ThrustForce(
            movingBody, movingBody.Transform.LocalForward, 30, movingBody.Transform.LocalLeft);

        simulation.RunFor(
            durationSeconds: 5,
            stepsPerSecond,
            beforeStep: _ => applier.ApplyForce(thrust),
            afterStep: AssertAttachmentHolds);
        return;

        void AssertAttachmentHolds(float _)
        {
            PhysicsAssert.PoseNear(position, orientation, movingBody, positionTolerance: 1e-4f);
            PhysicsAssert.AtRest(fixedBody);
        }
    }

    [Theory]
    [InlineData(60)]
    [InlineData(120)]
    public void PositionAndOrientationErrorsConvergeToTheOriginalRelativePose(int stepsPerSecond)
    {
        var fixedBody = PhysicsObjects.Body(mass: 0, position: new Vector3(-1, 2, 1),
            orientation: Quaternion.CreateFromYawPitchRoll(0.5f, 0.2f, -0.3f));
        var position = new Vector3(2, -1, 3);
        var orientation = Quaternion.CreateFromYawPitchRoll(-0.3f, 0.8f, 0.4f);
        var displacedBody = PhysicsObjects.Body(mass: 2, position: position, orientation: orientation);
        var joint = PhysicsObjects.Joint(fixedBody, displacedBody, new Vector3(0, 3, -2));
        var simulation = new PhysicsSimulation(fixedBody.GameObject, displacedBody.GameObject, joint);
        displacedBody.Transform.Position += new Vector3(0.5f, -0.2f, 0.4f);
        displacedBody.Transform.Orientation = Quaternion.Normalize(
            Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 2, 3)), 0.7f) * orientation);

        simulation.RunFor(durationSeconds: 5, stepsPerSecond);

        PhysicsAssert.PoseNear(position, orientation, displacedBody, positionTolerance: 1e-3f);
        PhysicsAssert.AtRest(displacedBody, tolerance: 1e-3f);
    }

    [Theory]
    [InlineData(60)]
    [InlineData(120)]
    public void RotatingDynamicBodiesPreserveAnchorsRelativeOrientationAndLinearMomentum(int stepsPerSecond)
    {
        var bodyA = PhysicsObjects.Body(
            mass: 2,
            position: -Vector3.UnitX,
            orientation: Quaternion.CreateFromYawPitchRoll(0.2f, 0.1f, 0.3f));
        var bodyB = PhysicsObjects.Body(
            mass: 3,
            position: Vector3.UnitX,
            orientation: Quaternion.CreateFromYawPitchRoll(-0.3f, 0.2f, 0.5f));
        var anchor = new Vector3(0, 0.5f, 0.2f);
        var localAnchorA = bodyA.Transform.InverseTransformPoint(anchor);
        var localAnchorB = bodyB.Transform.InverseTransformPoint(anchor);
        var relativeOrientation = Quaternion.Inverse(bodyA.Transform.Orientation) * bodyB.Transform.Orientation;

        var joint = PhysicsObjects.Joint(bodyA, bodyB, anchor);
        var simulation = new PhysicsSimulation(bodyA.GameObject, bodyB.GameObject, joint);
        bodyA.AngularVelocity = bodyB.AngularVelocity = Vector3.Normalize(new Vector3(1, 2, 3)) * 0.5f;
        bodyA.Velocity = Vector3.Cross(bodyA.AngularVelocity, bodyA.Transform.Position);
        bodyB.Velocity = Vector3.Cross(bodyB.AngularVelocity, bodyB.Transform.Position);
        var initialMomentum = PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB);

        simulation.RunFor(durationSeconds: 5, stepsPerSecond, afterStep: AssertJointInvariants);
        return;

        void AssertJointInvariants(float _)
        {
            var worldAnchorA = bodyA.Transform.TransformPoint(localAnchorA);
            var worldAnchorB = bodyB.Transform.TransformPoint(localAnchorB);
            var actualRelativeOrientation =
                Quaternion.Inverse(bodyA.Transform.Orientation) * bodyB.Transform.Orientation;
            var momentum = PhysicsMeasurements.CalculateLinearMomentum(bodyA, bodyB);

            PhysicsAssert.VectorNear(worldAnchorA, worldAnchorB, tolerance: 0.01f);
            PhysicsAssert.OrientationNear(relativeOrientation, actualRelativeOrientation);
            PhysicsAssert.VectorNear(initialMomentum, momentum, tolerance: 1e-4f);
        }
    }

    [Fact]
    public void ChainOfJointsRemainsAttachedWhenForceIsAppliedToTheLastBody()
    {
        var anchorBody = PhysicsObjects.Body(mass: 0);
        var middleBody = PhysicsObjects.Body(position: Vector3.UnitX);
        var endBody = PhysicsObjects.Body(position: Vector3.UnitX * 2);
        var firstJoint = PhysicsObjects.Joint(anchorBody, middleBody, new Vector3(0.5f, 1, 0));
        var secondJoint = PhysicsObjects.Joint(middleBody, endBody, new Vector3(1.5f, 1, 0));
        var simulation = new PhysicsSimulation(
            anchorBody.GameObject, middleBody.GameObject, endBody.GameObject, firstJoint, secondJoint);
        var applier = endBody.GameObject.GetComponent<ForceApplier>()!;

        var thrust = new ThrustForce(endBody, -endBody.Transform.LocalUp, 10, endBody.Transform.LocalLeft);

        simulation.RunFor(
            durationSeconds: 5,
            stepsPerSecond: 120,
            beforeStep: _ => applier.ApplyForce(thrust),
            afterStep: AssertChainStaysAttached);
        return;

        void AssertChainStaysAttached(float _)
        {
            PhysicsAssert.VectorNear(Vector3.UnitX, middleBody.Transform.Position, tolerance: 0.01f);
            PhysicsAssert.PoseNear(Vector3.UnitX * 2, Quaternion.Identity, endBody, positionTolerance: 0.01f);
        }
    }

    [Fact]
    public void JointBetweenTwoStaticBodiesDoesNotGenerateInvalidState()
    {
        var bodyA = PhysicsObjects.Body(mass: 0);
        var bodyB = PhysicsObjects.Body(mass: 0, position: Vector3.UnitX);
        var simulation = new PhysicsSimulation(bodyA.GameObject, bodyB.GameObject,
            PhysicsObjects.Joint(bodyA, bodyB, Vector3.UnitY));

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);

        PhysicsAssert.VectorNear(Vector3.Zero, bodyA.Transform.Position);
        PhysicsAssert.VectorNear(Vector3.UnitX, bodyB.Transform.Position);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyA.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyB.AngularVelocity);
    }

    [Fact]
    public void ZeroInertiaBodiesStillShareLinearMotionWithoutNaN()
    {
        var bodyA = PhysicsObjects.Body(mass: 1, inertia: 0);
        var bodyB = PhysicsObjects.Body(mass: 2, inertia: 0, position: Vector3.UnitX);
        var joint = PhysicsObjects.Joint(bodyA, bodyB, Vector3.UnitY);
        var simulation = new PhysicsSimulation(bodyA.GameObject, bodyB.GameObject, joint);
        bodyA.ApplyImpulse(Vector3.UnitY * 3, Vector3.Zero);

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);

        PhysicsAssert.VectorNear(Vector3.UnitY, bodyA.Velocity);
        PhysicsAssert.VectorNear(bodyA.Velocity, bodyB.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitX, bodyB.Transform.Position - bodyA.Transform.Position);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyA.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.Zero, bodyB.AngularVelocity);
    }
}