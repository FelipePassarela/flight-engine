using System.Numerics;
using FlightEngine.Physics.Forces;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class ForceGeneratorTests
{
    [Theory]
    [InlineData(0f)]
    [InlineData(0.5f)]
    [InlineData(5f)]
    public void GravityForceIsProportionalToMass(float mass)
    {
        var body = PhysicsObjects.Body(mass: mass);
        var force = new GravityForce(body, new Vector3(0, -10, 0));

        PhysicsAssert.VectorNear(new Vector3(0, -10 * mass, 0), force.CalculateWorldForce());
    }

    [Fact]
    public void ThrustFollowsTheBodyOrientation()
    {
        var body = PhysicsObjects.Body(orientation:
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI / 2));
        var force = new ThrustForce(body, Vector3.UnitZ, 10);

        PhysicsAssert.VectorNear(Vector3.UnitX * 10, force.CalculateWorldForce());
    }

    [Theory]
    [InlineData(0.5f, -5f)]
    [InlineData(1f, 0f)]
    [InlineData(2f, 10f)]
    public void SpringRestoresItsRestLengthWhenCompressedOrExtended(float distance, float expectedForce)
    {
        var a = PhysicsObjects.Body();
        var b = PhysicsObjects.Body(position: Vector3.UnitX * distance);
        var force = new SpringForce(a, b, Vector3.Zero, Vector3.Zero,
            stiffness: 10, damping: 0, restLength: 1);

        PhysicsAssert.VectorNear(Vector3.UnitX * expectedForce, force.CalculateWorldForce());
    }

    [Fact]
    public void SpringDampingOpposesRelativeAnchorVelocityIncludingRotation()
    {
        var a = PhysicsObjects.Body();
        var b = PhysicsObjects.Body(position: Vector3.UnitX * 2);
        b.AngularVelocity = Vector3.UnitZ * 3;
        // Both anchors are at local +Y. Rotation moves B's anchor toward A at 3 m/s.
        var force = new SpringForce(a, b, Vector3.UnitY, Vector3.UnitY,
            stiffness: 0, damping: 2, restLength: 2);

        PhysicsAssert.VectorNear(-Vector3.UnitX * 6, force.CalculateWorldForce());
    }

    [Fact]
    public void CoincidentSpringAnchorsDoNotProduceNaN()
    {
        var a = PhysicsObjects.Body();
        var b = PhysicsObjects.Body();
        var force = new SpringForce(a, b, Vector3.Zero, Vector3.Zero, 10, 1, 1);

        PhysicsAssert.VectorNear(Vector3.Zero, force.CalculateWorldForce());
    }

    [Fact]
    public void DragOpposesMotionAndScalesWithSpeedSquared()
    {
        var body = PhysicsObjects.Body();
        body.Velocity = Vector3.UnitX * 2;
        var force = new DragForce(body, referenceArea: 2, dragCoefficient: 0.5f, airDensity: 1);
        PhysicsAssert.VectorNear(-Vector3.UnitX * 2, force.CalculateWorldForce());

        body.Velocity *= 2;
        PhysicsAssert.VectorNear(-Vector3.UnitX * 8, force.CalculateWorldForce());
    }

    [Fact]
    public void WindExertsDragOnAStationaryBody()
    {
        var body = PhysicsObjects.Body();
        var force = new DragForce(body, referenceArea: 2, dragCoefficient: 0.5f,
            airDensity: 1, windVelocity: Vector3.UnitX * 2);

        PhysicsAssert.VectorNear(Vector3.UnitX * 2, force.CalculateWorldForce());
    }

    [Fact]
    public void BodyMovingWithTheWindExperiencesNoDrag()
    {
        var body = PhysicsObjects.Body();
        body.Velocity = Vector3.UnitZ * 3;
        var force = new DragForce(body, referenceArea: 2, windVelocity: body.Velocity);

        PhysicsAssert.VectorNear(Vector3.Zero, force.CalculateWorldForce());
    }
}
