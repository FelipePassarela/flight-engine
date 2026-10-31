using System.Numerics;
using FlightEngine.Application;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class AirplaneTests
{
    [Fact]
    public void FlightSceneRegistersAirplaneComponentAndItsParts()
    {
        var flightScene = new FlightScene();
        var airplane = Assert.Single(flightScene.Scene.GetComponents<Airplane>());

        Assert.Same(flightScene.Airplane, airplane);
        Assert.Same(airplane.Fuselage, airplane.GameObject);
        Assert.Same(airplane, airplane.Fuselage.GetComponent<Airplane>());

        var bodies = flightScene.Scene.GetComponents<RigidBody>().ToArray();
        Assert.Contains(bodies, body => body.GameObject == airplane.Fuselage);
        Assert.Contains(bodies, body => body.GameObject == airplane.LeftWing);
        Assert.Contains(bodies, body => body.GameObject == airplane.RightWing);

        var joints = flightScene.Scene.GetComponents<FixedJoint>().ToArray();
        Assert.Equal(2, joints.Length);
        Assert.All(joints, joint => Assert.Same(airplane.Fuselage, joint.BodyA.GameObject));
        Assert.Contains(joints, joint => joint.BodyB.GameObject == airplane.LeftWing);
        Assert.Contains(joints, joint => joint.BodyB.GameObject == airplane.RightWing);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(60)]
    [InlineData(120)]
    public void WingsStayAttachedUnderGravityThrustAndATorquePulse(int stepsPerSecond)
    {
        var (airplane, gameObjects) = AirplaneFactory.Create(new Vector3(2, 3, -4));
        var simulation = new PhysicsSimulation([.. gameObjects]);
        var fuselage = airplane.Fuselage.GetComponent<RigidBody>()!;
        var wings = new[] { airplane.LeftWing, airplane.RightWing }
            .Select(wing => wing.GetComponent<RigidBody>()!)
            .Select(body => (Body: body, LocalPosition: fuselage.Transform.InverseTransformPoint(body.Transform.Position)))
            .ToArray();
        var joints = gameObjects.SelectMany(go => go.GetComponents<FixedJoint>()).ToArray();

        Assert.Equal(2, joints.Length);
        Assert.Contains(joints, j => j.BodyA == fuselage && j.BodyB.GameObject == airplane.LeftWing);
        Assert.Contains(joints, j => j.BodyA == fuselage && j.BodyB.GameObject == airplane.RightWing);
        Assert.Single(gameObjects.SelectMany(go => go.GetComponents<FlightInputController>()));

        var applier = airplane.Fuselage.GetComponent<ForceApplier>()!;
        var centralThrust = new ThrustForce(fuselage, fuselage.Transform.LocalForward, 10);
        var offCenterThrust = new ThrustForce(fuselage, fuselage.Transform.LocalForward, 10,
            fuselage.Transform.LocalLeft);

        simulation.RunFor(durationSeconds: 5, stepsPerSecond,
            beforeStep: step =>
            {
                // A short off-center pulse initiates rotation, then thrust acts at the center.
                var thrust = step < stepsPerSecond / 5 ? offCenterThrust : centralThrust;
                applier.AddForce(thrust);
            },
            afterStep: () =>
            {
                foreach (var (wing, localPosition) in wings)
                    PhysicsAssert.PoseNear(fuselage.Transform.TransformPoint(localPosition),
                        fuselage.Transform.Orientation, wing, positionTolerance: 0.01f);
            });
    }

    [Fact]
    public void UniformGravityDoesNotPitchTheAirplane()
    {
        var (_, gameObjects) = AirplaneFactory.Create(new Vector3(0, 3, 0));
        var simulation = new PhysicsSimulation([.. gameObjects]);

        const int duration = 5;
        simulation.RunFor(duration, stepsPerSecond: 120);

        foreach (var body in gameObjects.SelectMany(go => go.GetComponents<RigidBody>()))
        {
            PhysicsAssert.VectorNear(Gravity.MoonGravityAcceleration * duration, body.Velocity, 1e-3f);
            PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity, 1e-4f);
            PhysicsAssert.OrientationNear(Quaternion.Identity, body.Transform.Orientation);
        }
    }
}