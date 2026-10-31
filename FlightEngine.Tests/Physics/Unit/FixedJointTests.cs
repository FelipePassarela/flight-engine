using System.Numerics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Unit;

[Trait("Category", "Unit")]
public class FixedJointTests
{
    [Fact]
    public void JointCannotConnectABodyToItself()
    {
        var body = PhysicsObjects.Body();
        var joint = new FixedJoint(body, body, Vector3.Zero);

        Assert.Throws<ArgumentException>(joint.OnInitialize);
    }

    [Fact]
    public void SolvingBeforePreparationDoesNotChangeVelocities()
    {
        var a = PhysicsObjects.Body();
        var b = PhysicsObjects.Body();
        a.Velocity = Vector3.UnitX;
        b.Velocity = Vector3.UnitY;
        var joint = new FixedJoint(a, b, Vector3.Zero);
        joint.OnInitialize();
        joint.SolveVelocity(1f / 60);

        PhysicsAssert.VectorNear(Vector3.UnitX, a.Velocity);
        PhysicsAssert.VectorNear(Vector3.UnitY, b.Velocity);
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(-1f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    public void InvalidPreparationDisablesAPreviouslyPreparedJoint(float deltaTime)
    {
        var a = PhysicsObjects.Body();
        var b = PhysicsObjects.Body();
        var joint = new FixedJoint(a, b, Vector3.Zero);
        joint.OnInitialize();
        joint.Prepare(1f / 60);
        a.Velocity = Vector3.UnitX;
        joint.Prepare(deltaTime);
        joint.SolveVelocity(1f / 60);

        PhysicsAssert.VectorNear(Vector3.UnitX, a.Velocity);
        PhysicsAssert.VectorNear(Vector3.Zero, b.Velocity);
    }

    [Fact]
    public void EquivalentQuaternionSignsDoNotGenerateAnAngularImpulse()
    {
        var orientation = Quaternion.CreateFromYawPitchRoll(0.4f, 0.2f, -0.3f);
        var a = PhysicsObjects.Body(mass: 0);
        var b = PhysicsObjects.Body(orientation: orientation);
        var joint = new FixedJoint(a, b, Vector3.Zero);
        joint.OnInitialize();
        b.Transform.Orientation = Quaternion.Negate(orientation);
        joint.Prepare(1f / 60);
        joint.SolveVelocity(1f / 60);

        PhysicsAssert.VectorNear(Vector3.Zero, b.AngularVelocity);
    }
}
