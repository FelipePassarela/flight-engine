using System.Numerics;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class RigidBodyTests
{
    [Fact]
    public void ImpulseAtCenterChangesLinearVelocityWithoutRotation()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4);
        body.Velocity = Vector3.UnitX;
        body.AddImpulse(new Vector3(0, 6, 0), Vector3.Zero);

        PhysicsAssert.VectorNear(new Vector3(1, 3, 0), body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }

    [Fact]
    public void OffCenterImpulseUsesTheRotatedApplicationPoint()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4,
            orientation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        // Local +X points along world +Y, so r x P = +Y x +Z * 8 = +X * 8.
        body.AddImpulse(Vector3.UnitZ * 8, Vector3.UnitX);

        PhysicsAssert.VectorNear(Vector3.UnitZ * 4, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitX * 2, body.AngularVelocity);
    }

    [Fact]
    public void AngularImpulseDoesNotChangeLinearVelocity()
    {
        var body = PhysicsObjects.Body(inertia: 4);
        body.AddAngularImpulse(new Vector3(0, 8, 0));

        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);
        PhysicsAssert.VectorNear(new Vector3(0, 2, 0), body.AngularVelocity);
    }

    [Fact]
    public void ForceAccumulatesUntilVelocityIntegrationAndCanBeCleared()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4);
        body.AddForce(Vector3.UnitZ * 8, Vector3.UnitY);
        body.AddForce(Vector3.UnitZ * 8, Vector3.UnitY);
        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);

        body.IntegrateVelocities(0.5f);
        PhysicsAssert.VectorNear(Vector3.UnitZ * 4, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitX * 2, body.AngularVelocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.Transform.Position);

        body.ClearAccumulators();
        body.IntegrateVelocities(0.5f);
        PhysicsAssert.VectorNear(Vector3.UnitZ * 4, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitX * 2, body.AngularVelocity);
    }

    [Fact]
    public void VelocityAtPointIncludesRotationInWorldSpace()
    {
        var body = PhysicsObjects.Body(orientation:
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        body.Velocity = Vector3.UnitX * 2;
        body.AngularVelocity = Vector3.UnitZ * 3;

        PhysicsAssert.VectorNear(new Vector3(-1, 0, 0), body.GetVelocityAtPoint(Vector3.UnitX));
    }

    [Fact]
    public void TransformIntegrationAppliesWorldSpaceAngularVelocity()
    {
        var initial = Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.6f);
        var body = PhysicsObjects.Body(position: Vector3.One, orientation: initial);
        body.Velocity = Vector3.UnitX * 2;
        body.AngularVelocity = Vector3.UnitZ * MathF.PI;

        body.IntegrateTransform(0.5f);

        PhysicsAssert.VectorNear(new Vector3(2, 1, 1), body.Transform.Position);
        PhysicsAssert.OrientationNear(
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2) * initial,
            body.Transform.Orientation);
    }

    [Fact]
    public void StopMotionAlsoClearsPendingForceAndTorque()
    {
        var body = PhysicsObjects.Body();
        body.Velocity = body.AngularVelocity = Vector3.One;
        body.AddForce(Vector3.UnitY * 10, Vector3.UnitX);
        body.StopMotion();
        body.IntegrateVelocities(1);

        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }

    [Fact]
    public void ZeroInertiaDoesNotProduceNaNWhenReceivingTorque()
    {
        var body = PhysicsObjects.Body(inertia: 0);
        body.AddForce(Vector3.UnitY * 4, Vector3.UnitX);
        body.AddAngularImpulse(Vector3.One);
        body.IntegrateVelocities(0.5f);

        PhysicsAssert.VectorNear(Vector3.UnitY * 2, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }
}
