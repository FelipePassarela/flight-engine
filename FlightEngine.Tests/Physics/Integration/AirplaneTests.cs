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
        Assert.Single(flightScene.Scene.GetComponents<FlightInputController>());

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
        var (airplane, simulation, _) = CreateAirplaneSimulation(new Vector3(2, 3, -4));
        var fuselage = airplane.Fuselage.GetComponent<RigidBody>()!;
        var wings = CaptureWingPositions(airplane);
        var applier = airplane.Fuselage.GetComponent<ForceApplier>()!;
        var offCenterThrust = new ThrustForce(fuselage, fuselage.Transform.LocalForward, 10,
            fuselage.Transform.LocalLeft);

        simulation.RunFor(durationSeconds: 5, stepsPerSecond,
            beforeStep: step =>
            {
                // A short off-center pulse initiates rotation, then thrust acts at the center.
                if (step < stepsPerSecond / 5)
                    applier.ApplyForce(offCenterThrust);
                else
                    airplane.ApplyThrust(fuselage.Transform.LocalForward);
            },
            afterStep: () => AssertWingsStayAttached(fuselage, wings));
    }

    [Fact]
    public void CenteredThrustAcceleratesTheAirplaneInTheRequestedDirection()
    {
        var (airplane, simulation, bodies) = CreateAirplaneSimulation(Vector3.Zero);
        foreach (var body in bodies)
            body.GameObject.GetComponent<Gravity>()!.Active = false;

        var thrustDirection = Vector3.UnitZ;
        const float thrustNewtons = 10f;
        const int durationSeconds = 1;

        simulation.RunFor(durationSeconds, stepsPerSecond: 60,
            beforeStep: _ => airplane.ApplyThrust(thrustDirection));

        var totalMass = bodies.Sum(body => body.Mass);
        var expectedVelocity = thrustDirection * (thrustNewtons * durationSeconds / totalMass);
        var centerOfMassVelocity = PhysicsMeasurements.CalculateLinearMomentum(bodies) / totalMass;
        PhysicsAssert.VectorNear(expectedVelocity, centerOfMassVelocity, 1e-3f);
    }

    [Theory]
    [InlineData(true, 30)]
    [InlineData(true, 60)]
    [InlineData(true, 120)]
    [InlineData(false, 30)]
    [InlineData(false, 60)]
    [InlineData(false, 120)]
    public void UpwardPulseAtWingCenterRollsTheWholeAirplane(bool useLeftWing, int stepsPerSecond)
    {
        var (airplane, simulation, _) = CreateAirplaneSimulation(new Vector3(2, 3, -4));
        var fuselage = airplane.Fuselage.GetComponent<RigidBody>()!;
        var wings = CaptureWingPositions(airplane);
        var targetWing = (useLeftWing ? airplane.LeftWing : airplane.RightWing).GetComponent<RigidBody>()!;
        var wingLift = new ThrustForce(targetWing, targetWing.Transform.LocalUp, 5, targetWing.CenterOfMass);

        simulation.RunFor(durationSeconds: 1, stepsPerSecond,
            beforeStep: step =>
            {
                if (step < stepsPerSecond / 5)
                    targetWing.ApplyForce(wingLift.CalculateWorldForce(), wingLift.LocalApplicationPoint);
            },
            afterStep: () => AssertWingsStayAttached(fuselage, wings));

        var expectedRollAxis = useLeftWing ? Vector3.UnitZ : -Vector3.UnitZ;
        PhysicsAssert.RotatingAroundAxis(fuselage, expectedRollAxis,
            minimumAxialSpeed: 0.25f, maximumOffAxisSpeed: 1e-3f);
        PhysicsAssert.RotatedByAtLeast(Quaternion.Identity, fuselage.Transform.Orientation, minimumAngleDegrees: 17f);
        foreach (var (wing, _) in wings)
            PhysicsAssert.VectorNear(fuselage.AngularVelocity, wing.AngularVelocity, 1e-3f);
    }

    [Theory]
    [InlineData(-1f, 30)]
    [InlineData(-1f, 60)]
    [InlineData(-1f, 120)]
    [InlineData(0f, 30)]
    [InlineData(0f, 60)]
    [InlineData(0f, 120)]
    [InlineData(1f, 30)]
    [InlineData(1f, 60)]
    [InlineData(1f, 120)]
    public void ContinuousRollPreservesCenterOfMassMotion(float rollInput, int stepsPerSecond)
    {
        var position = new Vector3(2, 3, -4);
        var initialOrientation = Quaternion.CreateFromYawPitchRoll(0.6f, -0.3f, 0.4f);
        var initialVelocity = new Vector3(2, -1, 3);
        var (airplane, simulation, bodies) = CreateAirplaneSimulation(position);
        var fuselage = airplane.Fuselage.GetComponent<RigidBody>()!;

        // Rotate the initialized assembly as a whole, preserving the joints' local anchors.
        foreach (var body in bodies)
        {
            body.Transform.Position = position + Vector3.Transform(body.Transform.Position - position, initialOrientation);
            body.Transform.Orientation = initialOrientation;
            body.Velocity = initialVelocity;
            body.GameObject.GetComponent<Gravity>()!.Active = false;
        }

        var initialCenter = PhysicsMeasurements.CalculateCenterOfMass(bodies);
        var initialMomentum = PhysicsMeasurements.CalculateLinearMomentum(bodies);
        var rollAxis = fuselage.Transform.WorldForward;
        var completedSteps = 0;

        simulation.RunFor(durationSeconds: 2, stepsPerSecond,
            beforeStep: _ => airplane.ApplyRoll(rollInput),
            afterStep: () =>
            {
                completedSteps++;
                var elapsedTime = (float)completedSteps / stepsPerSecond;
                var expectedCenter = initialCenter + initialVelocity * elapsedTime;
                PhysicsAssert.VectorNear(initialMomentum, PhysicsMeasurements.CalculateLinearMomentum(bodies), 1e-3f);
                PhysicsAssert.VectorNear(expectedCenter, PhysicsMeasurements.CalculateCenterOfMass(bodies), 1e-3f);
            });

        if (rollInput == 0f)
        {
            PhysicsAssert.VectorNear(Vector3.Zero, fuselage.AngularVelocity);
            PhysicsAssert.OrientationNear(initialOrientation, fuselage.Transform.Orientation);
        }
        else
        {
            var expectedRollAxis = rollAxis * rollInput;
            PhysicsAssert.RotatingAroundAxis(fuselage, expectedRollAxis,
                minimumAxialSpeed: 1f, maximumOffAxisSpeed: 1e-3f);
            PhysicsAssert.RotatedByAtLeast(
                initialOrientation, fuselage.Transform.Orientation, minimumAngleDegrees: 17f);
        }

        foreach (var body in bodies)
            PhysicsAssert.OrientationNear(fuselage.Transform.Orientation, body.Transform.Orientation);
    }

    [Fact]
    public void UniformGravityDoesNotPitchTheAirplane()
    {
        var (_, simulation, bodies) = CreateAirplaneSimulation(new Vector3(0, 3, 0));

        const int duration = 5;
        simulation.RunFor(duration, stepsPerSecond: 120);

        foreach (var body in bodies)
        {
            PhysicsAssert.VectorNear(Gravity.MoonGravityAcceleration * duration, body.Velocity, 1e-3f);
            PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity, 1e-4f);
            PhysicsAssert.OrientationNear(Quaternion.Identity, body.Transform.Orientation);
        }
    }

    private static (Airplane Airplane, PhysicsSimulation Simulation, RigidBody[] Bodies) CreateAirplaneSimulation(
        Vector3 position)
    {
        var (airplane, gameObjects) = AirplaneFactory.Create(position);
        var simulation = new PhysicsSimulation([.. gameObjects]);
        var bodies = gameObjects.SelectMany(go => go.GetComponents<RigidBody>()).ToArray();
        return (airplane, simulation, bodies);
    }

    private static (RigidBody Body, Vector3 LocalPosition)[] CaptureWingPositions(Airplane airplane)
    {
        return new[] { airplane.LeftWing, airplane.RightWing }
            .Select(wing => wing.GetComponent<RigidBody>()!)
            .Select(body => (Body: body,
                LocalPosition: airplane.Fuselage.Transform.InverseTransformPoint(body.Transform.Position)))
            .ToArray();
    }

    private static void AssertWingsStayAttached(
        RigidBody fuselage, (RigidBody Body, Vector3 LocalPosition)[] wings)
    {
        foreach (var (wing, localPosition) in wings)
            PhysicsAssert.PoseNear(fuselage.Transform.TransformPoint(localPosition),
                fuselage.Transform.Orientation, wing, positionTolerance: 0.01f);
    }
}
