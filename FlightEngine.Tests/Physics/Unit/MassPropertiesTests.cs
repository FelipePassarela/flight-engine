using System.Numerics;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

// Verify the internal mass properties through the public RigidBody API.
[Trait("Category", "Unit")]
public class MassPropertiesTests
{
    [Theory]
    [InlineData(0f, 0f)]
    [InlineData(0.5f, 2f)]
    [InlineData(4f, 0.25f)]
    public void MassAndInverseMassCanBeUpdatedInBothDirections(float mass, float inverseMass)
    {
        var body = new RigidBody { Mass = mass };
        Assert.Equal(inverseMass, body.InverseMass);
        Assert.Equal(mass == 0, body.HasInfiniteMass);

        body.Mass = 10;
        body.InverseMass = inverseMass;
        Assert.Equal(mass, body.Mass);
        Assert.Equal(inverseMass, body.InverseMass);
    }

    [Theory]
    [InlineData(0f, 0f)]
    [InlineData(0.5f, 2f)]
    [InlineData(4f, 0.25f)]
    public void InertiaAndInverseInertiaCanBeUpdatedInBothDirections(float inertia, float inverseInertia)
    {
        var body = new RigidBody { MomentOfInertia = inertia };
        Assert.Equal(inverseInertia, body.InverseMomentOfInertia);

        body.MomentOfInertia = 10;
        body.InverseMomentOfInertia = inverseInertia;
        Assert.Equal(inertia, body.MomentOfInertia);
        Assert.Equal(inverseInertia, body.InverseMomentOfInertia);
    }

    [Theory]
    [InlineData(-1f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(float.NegativeInfinity)]
    public void InvalidPropertiesAreRejectedWithoutChangingExistingValues(float invalid)
    {
        var body = new RigidBody(mass: 2, momentOfInertia: 4);
        Assert.Throws<ArgumentOutOfRangeException>(() => new RigidBody(mass: invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => new RigidBody(momentOfInertia: invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => body.Mass = invalid);
        Assert.Throws<ArgumentOutOfRangeException>(() => body.InverseMass = invalid);
        Assert.Throws<ArgumentOutOfRangeException>(() => body.MomentOfInertia = invalid);
        Assert.Throws<ArgumentOutOfRangeException>(() => body.InverseMomentOfInertia = invalid);
        Assert.Equal(2, body.Mass);
        Assert.Equal(4, body.MomentOfInertia);
    }

    [Fact]
    public void StaticBodyKeepsItsStoredInertiaButDoesNotRespondToImpulses()
    {
        var body = PhysicsObjects.Body(mass: 0, inertia: 4);
        body.InverseMomentOfInertia = 0.5f;
        body.AddImpulse(Vector3.UnitY, Vector3.UnitX);
        body.AddAngularImpulse(Vector3.UnitZ);

        Assert.Equal(0, body.InverseMomentOfInertia);
        Assert.Equal(2, body.MomentOfInertia);
        PhysicsAssert.VectorNear(Vector3.Zero, body.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity);

        body.Mass = 1;
        Assert.Equal(0.5f, body.InverseMomentOfInertia);
    }
}
