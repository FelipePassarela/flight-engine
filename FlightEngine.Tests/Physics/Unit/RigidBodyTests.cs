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
        body.ApplyImpulse(new Vector3(0, 6, 0), Vector3.Zero);

        PhysicsAssert.VectorNear(new Vector3(1, 3, 0), body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }

    [Fact]
    public void OffCenterImpulseUsesTheRotatedApplicationPoint()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4,
            orientation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        // Local +X points along world +Y, so r x P = +Y x +Z * 8 = +X * 8.
        body.ApplyImpulse(Vector3.UnitZ * 8, Vector3.UnitX);

        PhysicsAssert.VectorNear(Vector3.UnitZ * 4, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitX * 2, body.AngularVelocity);
    }

    [Theory]
    [InlineData(1f, 1f, 1f)]
    [InlineData(2f, 2f, 2f)]
    [InlineData(0.5f, 0.5f, 0.5f)]
    [InlineData(2f, 3f, 4f)]
    public void OffCenterImpulseUsesScaleAndRotationWithoutTranslation(float scaleX, float scaleY, float scaleZ)
    {
        const float mass = 2f;
        const float inertia = 4f;
        var scale = new Vector3(scaleX, scaleY, scaleZ);
        var body = PhysicsObjects.Body(mass, inertia, position: new Vector3(5, -2, 3),
            orientation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        body.Transform.Scale = scale;
        var initialAngularVelocity = new Vector3(1, -2, 3);
        body.AngularVelocity = initialAngularVelocity;
        var impulse = new Vector3(4, -6, 8);
        var localApplicationPoint = new Vector3(1, 2, 3);

        body.ApplyImpulse(impulse, localApplicationPoint);

        var leverArm = ExpectedWorldOffsetAfterQuarterTurnAboutZ(localApplicationPoint, scale);
        var angularImpulse = Vector3.Cross(leverArm, impulse);
        var expectedAngularVelocity = initialAngularVelocity + angularImpulse / inertia;
        PhysicsAssert.VectorNear(impulse / mass, body.Velocity);
        PhysicsAssert.VectorNear(expectedAngularVelocity, body.AngularVelocity);
    }

    [Theory]
    [InlineData(1f, 1f, 1f)]
    [InlineData(2f, 2f, 2f)]
    [InlineData(0.5f, 0.5f, 0.5f)]
    [InlineData(2f, 3f, 4f)]
    public void OffCenterForceUsesScaleAndRotationWithoutTranslation(float scaleX, float scaleY, float scaleZ)
    {
        const float mass = 2f;
        const float inertia = 4f;
        const float deltaTime = 0.5f;
        var scale = new Vector3(scaleX, scaleY, scaleZ);
        var body = PhysicsObjects.Body(mass, inertia, position: new Vector3(5, -2, 3),
            orientation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        body.Transform.Scale = scale;
        var force = new Vector3(4, -6, 8);
        var localApplicationPoint = new Vector3(1, 2, 3);

        body.ApplyForce(force, localApplicationPoint);
        PhysicsAssert.AtRest(body);
        body.IntegrateVelocities(deltaTime);

        var leverArm = ExpectedWorldOffsetAfterQuarterTurnAboutZ(localApplicationPoint, scale);
        var torque = Vector3.Cross(leverArm, force);
        var expectedAngularVelocity = torque / inertia * deltaTime;
        var expectedLinearVelocity = force / mass * deltaTime;
        PhysicsAssert.VectorNear(expectedLinearVelocity, body.Velocity);
        PhysicsAssert.VectorNear(expectedAngularVelocity, body.AngularVelocity);
    }

    [Fact]
    public void AngularImpulseDoesNotChangeLinearVelocity()
    {
        var body = PhysicsObjects.Body(inertia: 4);
        body.ApplyAngularImpulse(new Vector3(0, 8, 0));

        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);
        PhysicsAssert.VectorNear(new Vector3(0, 2, 0), body.AngularVelocity);
    }

    [Fact]
    public void ForceAccumulatesUntilVelocityIntegrationAndCanBeCleared()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4);
        body.ApplyForce(Vector3.UnitZ * 8, Vector3.UnitY);
        body.ApplyForce(Vector3.UnitZ * 8, Vector3.UnitY);
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

    [Theory]
    [InlineData(1f, 1f, 1f)]
    [InlineData(2f, 2f, 2f)]
    [InlineData(0.5f, 0.5f, 0.5f)]
    [InlineData(2f, 3f, 4f)]
    public void VelocityAtPointUsesScaleAndRotationWithoutTranslation(float scaleX, float scaleY, float scaleZ)
    {
        var body = PhysicsObjects.Body(position: new Vector3(5, -2, 3),
            orientation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2));
        var scale = new Vector3(scaleX, scaleY, scaleZ);
        var linearVelocity = new Vector3(2, -1, 4);
        var angularVelocity = new Vector3(1, 2, 3);
        body.Transform.Scale = scale;
        body.Velocity = linearVelocity;
        body.AngularVelocity = angularVelocity;
        var localPoint = new Vector3(1, 2, 3);

        var pointVelocity = body.GetVelocityAtPoint(localPoint);

        var worldOffset = ExpectedWorldOffsetAfterQuarterTurnAboutZ(localPoint, scale);
        var rotationalVelocity = Vector3.Cross(angularVelocity, worldOffset);
        var expectedPointVelocity = linearVelocity + rotationalVelocity;
        PhysicsAssert.VectorNear(expectedPointVelocity, pointVelocity);
        PhysicsAssert.VectorNear(linearVelocity, body.GetVelocityAtPoint(Vector3.Zero));
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
        body.ApplyForce(Vector3.UnitY * 10, Vector3.UnitX);
        body.StopMotion();
        body.IntegrateVelocities(1);

        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }

    [Fact]
    public void ZeroInertiaDoesNotProduceNaNWhenReceivingTorque()
    {
        var body = PhysicsObjects.Body(inertia: 0);
        body.ApplyForce(Vector3.UnitY * 4, Vector3.UnitX);
        body.ApplyAngularImpulse(Vector3.One);
        body.IntegrateVelocities(0.5f);

        PhysicsAssert.VectorNear(Vector3.UnitY * 2, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);
    }

    private static Vector3 ExpectedWorldOffsetAfterQuarterTurnAboutZ(Vector3 localOffset, Vector3 scale)
    {
        var scaledOffset = localOffset * scale;
        // A positive quarter turn about Z maps +X to +Y and +Y to -X.
        return new Vector3(-scaledOffset.Y, scaledOffset.X, scaledOffset.Z);
    }
}
