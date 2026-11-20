using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Forces;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class PhysicsWorldTests
{
    [Fact]
    public void QueuedForcesAffectTheCurrentStepAndAreConsumedExactlyOnce()
    {
        var body = PhysicsObjects.Body(mass: 2, inertia: 4);
        var simulation = new PhysicsSimulation(body.GameObject);
        var applier = body.GameObject.GetComponent<ForceApplier>()!;
        applier.ApplyForce(new ThrustForce(body, Vector3.UnitZ, 8, Vector3.UnitY));
        applier.ApplyForce(new GravityForce(body, new Vector3(0, -4, 0)));

        simulation.Step(0.25f);

        PhysicsAssert.VectorNear(new Vector3(0, -1, 1), body.Velocity);
        PhysicsAssert.VectorNear(new Vector3(0, -0.25f, 0.25f), body.Transform.Position);
        PhysicsAssert.VectorNear(Vector3.UnitX * 0.5f, body.AngularVelocity);

        simulation.Step(0.25f);

        PhysicsAssert.VectorNear(new Vector3(0, -1, 1), body.Velocity);
        PhysicsAssert.VectorNear(new Vector3(0, -0.5f, 0.5f), body.Transform.Position);
        PhysicsAssert.VectorNear(Vector3.UnitX * 0.5f, body.AngularVelocity);
        PhysicsAssert.OrientationNear(Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.25f), body.Transform.Orientation);
        Assert.Null(applier.GetForceGenerator<ThrustForce>());
    }

    [Theory]
    [InlineData(30)]
    [InlineData(60)]
    [InlineData(120)]
    public void FreeFallHasMassIndependentAcceleration(int stepsPerSecond)
    {
        var gravity = new Vector3(0, -9.81f, 0);
        var lightBody = PhysicsObjects.Body(mass: 0.5f, gravity: gravity);
        var heavyBody = PhysicsObjects.Body(mass: 8, gravity: gravity);
        var simulation = new PhysicsSimulation(lightBody.GameObject, heavyBody.GameObject);
        const int duration = 3;
        var deltaTime = 1f / stepsPerSecond;

        simulation.RunFor(duration, stepsPerSecond);

        PhysicsAssert.VectorNear(gravity * duration, lightBody.Velocity, 1e-3f);
        PhysicsAssert.VectorNear(lightBody.Velocity, heavyBody.Velocity);
        PhysicsAssert.VectorNear(lightBody.Transform.Position, heavyBody.Transform.Position);
        // Semi-implicit Euler has a first-order position error of |g| * T * dt / 2.
        var analyticalPosition = gravity * (0.5f * duration * duration);
        var tolerance = gravity.Length() * duration * deltaTime / 2 + 1e-3f;
        PhysicsAssert.VectorNear(
            analyticalPosition, lightBody.Transform.Position, tolerance);
        PhysicsAssert.VectorNear(Vector3.Zero, lightBody.AngularVelocity);
    }

    [Fact]
    public void StaticBodyRemainsAtItsPoseUnderGravityForcesAndImpulses()
    {
        var position = new Vector3(1, 2, 3);
        var orientation = Quaternion.CreateFromYawPitchRoll(0.3f, -0.2f, 0.4f);
        var body = PhysicsObjects.Body(
            mass: 0, position: position, orientation: orientation, gravity: Gravity.EarthGravityAcceleration);
        var simulation = new PhysicsSimulation(body.GameObject);
        body.Velocity = body.AngularVelocity = Vector3.One;

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60, beforeStep: ApplyOffCenterLoads);

        PhysicsAssert.PoseNear(position, orientation, body);
        PhysicsAssert.AtRest(body);
        return;

        void ApplyOffCenterLoads(int _)
        {
            body.ApplyImpulse(Vector3.UnitY * 10, body.Transform.LocalLeft);
            body.ApplyForce(Vector3.UnitZ * 10, body.Transform.LocalUp);
        }
    }

    [Fact]
    public void DisablingGravityStopsAccelerationWithoutStoppingExistingMotion()
    {
        var body = PhysicsObjects.Body(gravity: new Vector3(0, -10, 0));
        var simulation = new PhysicsSimulation(body.GameObject);
        simulation.Step(0.1f);
        body.GameObject.GetComponent<Gravity>()!.Active = false;
        simulation.Step(0.1f);

        PhysicsAssert.VectorNear(-Vector3.UnitY, body.Velocity);
        PhysicsAssert.VectorNear(-Vector3.UnitY * 0.2f, body.Transform.Position);
    }

    [Theory]
    [InlineData(60)]
    [InlineData(120)]
    public void DampedSpringSettlesWithoutMovingTheCombinedCenterOfMass(int stepsPerSecond)
    {
        var lightBody = PhysicsObjects.Body(mass: 1, position: -Vector3.UnitX);
        var heavyBody = PhysicsObjects.Body(mass: 2, position: Vector3.UnitX * 2);
        var spring = new GameObject([new Spring(lightBody, heavyBody, stiffness: 8, damping: 2, restLength: 1)]);
        var simulation = new PhysicsSimulation(lightBody.GameObject, heavyBody.GameObject, spring);
        var initialCenter = PhysicsMeasurements.CalculateCenterOfMass(lightBody, heavyBody);

        simulation.RunFor(durationSeconds: 10, stepsPerSecond, afterStep: AssertCenterOfMassStaysFixed);

        Assert.InRange(Vector3.Distance(lightBody.Transform.Position, heavyBody.Transform.Position), 0.999f, 1.001f);
        PhysicsAssert.VectorNear(Vector3.Zero, lightBody.Velocity, 1e-3f);
        PhysicsAssert.VectorNear(Vector3.Zero, heavyBody.Velocity, 1e-3f);
        return;

        void AssertCenterOfMassStaysFixed(float _)
        {
            var momentum = PhysicsMeasurements.CalculateLinearMomentum(lightBody, heavyBody);
            var center = PhysicsMeasurements.CalculateCenterOfMass(lightBody, heavyBody);
            PhysicsAssert.VectorNear(Vector3.Zero, momentum, tolerance: 1e-4f);
            PhysicsAssert.VectorNear(initialCenter, center, tolerance: 1e-4f);
        }
    }
}