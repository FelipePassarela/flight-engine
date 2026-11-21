using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class HingeLimitTests
{
    private const float MinimumAngle = -0.3f;
    private const float MaximumAngle = 0.4f;
    private const float AngleTolerance = 1e-4f;

    [Fact]
    public void InsideTheLimitsTheJointAllowsFreeRotation()
    {
        var (body, simulation) = CreateCenteredHinge();
        const float speed = 0.2f;
        const int durationSeconds = 1;
        body.AngularVelocity = speed * Vector3.UnitX;

        simulation.RunFor(durationSeconds, stepsPerSecond: 60);

        AssertAngle(speed * durationSeconds, body);
        PhysicsAssert.VectorNear(speed * Vector3.UnitX, body.AngularVelocity);
    }

    [Theory]
    [InlineData(-1f, 30)]
    [InlineData(1f, 30)]
    [InlineData(-1f, 120)]
    [InlineData(1f, 120)]
    public void HighSpeedRotationStopsAtEitherLimitBeforeCrossingIt(float direction, int stepsPerSecond)
    {
        var (body, simulation) = CreateCenteredHinge();
        var expectedStop = direction < 0f ? MinimumAngle : MaximumAngle;
        body.AngularVelocity = (24f * direction) * Vector3.UnitX;

        simulation.RunFor(durationSeconds: 1, stepsPerSecond,
            afterStep: _ => Assert.InRange(ReadAngle(body), MinimumAngle - AngleTolerance, MaximumAngle + AngleTolerance));

        AssertAngle(expectedStop, body);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity, tolerance: AngleTolerance);
    }

    [Theory]
    [InlineData(-1f)]
    [InlineData(1f)]
    public void RotationCanReverseAwayFromEitherStop(float direction)
    {
        var (body, simulation) = CreateCenteredHinge();
        var stop = direction < 0f ? MinimumAngle : MaximumAngle;
        body.AngularVelocity = (24f * direction) * Vector3.UnitX;
        simulation.RunFor(durationSeconds: 1, stepsPerSecond: 60);
        AssertAngle(stop, body);

        const float inwardSpeed = 0.2f;
        const int returnDurationSeconds = 1;
        body.AngularVelocity = (-direction * inwardSpeed) * Vector3.UnitX;
        simulation.RunFor(returnDurationSeconds, stepsPerSecond: 60);

        var expectedAngle = stop - direction * inwardSpeed * returnDurationSeconds;
        AssertAngle(expectedAngle, body);
    }

    [Theory]
    [InlineData(-0.7f, MinimumAngle)]
    [InlineData(0.7f, MaximumAngle)]
    public void RotationOutsideTheRangeReturnsToTheViolatedStop(float displacedAngle, float expectedStop)
    {
        var (body, simulation) = CreateCenteredHinge();
        simulation.Step(1f / 60f);
        body.Transform.Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, displacedAngle);

        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);

        AssertAngle(expectedStop, body);
        PhysicsAssert.VectorNear(Vector3.Zero, body.AngularVelocity, tolerance: AngleTolerance);
    }

    [Fact]
    public void TheStartingPoseDefinesZeroWithRotatedAndTwistedBodies()
    {
        var localAxis = Vector3.UnitX;
        var fixedOrientation = Quaternion.CreateFromYawPitchRoll(0.5f, -0.3f, 0.4f);
        var worldAxis = Vector3.Transform(localAxis, fixedOrientation);
        var initialTwist = Quaternion.CreateFromAxisAngle(worldAxis, 0.7f);
        var movingOrientation = initialTwist * fixedOrientation;
        var (movingBody, simulation) = CreateCenteredHinge(fixedOrientation, movingOrientation);
        movingBody.AngularVelocity = 24f * worldAxis;

        simulation.RunFor(durationSeconds: 1, stepsPerSecond: 60);

        var stopRotation = Quaternion.CreateFromAxisAngle(worldAxis, MaximumAngle);
        var expectedOrientation = stopRotation * movingOrientation;
        PhysicsAssert.OrientationNear(expectedOrientation, movingBody.Transform.Orientation);
    }

    [Fact]
    public void AligningTiltedAxesPreservesTheNeutralAngle()
    {
        var tiltAxis = Vector3.Normalize(new Vector3(0f, 1f, -1f));
        var initialTilt = Quaternion.CreateFromAxisAngle(tiltAxis, 0.9f);
        var (body, simulation) = CreateCenteredHinge(movingOrientation: initialTilt);
        simulation.RunFor(durationSeconds: 2, stepsPerSecond: 60);
        PhysicsAssert.OrientationNear(Quaternion.Identity, body.Transform.Orientation);

        body.AngularVelocity = 24f * Vector3.UnitX;
        simulation.RunFor(durationSeconds: 1, stepsPerSecond: 60);

        AssertAngle(MaximumAngle, body);
    }

    private static (RigidBody Body, PhysicsSimulation Simulation) CreateCenteredHinge(
        Quaternion? fixedOrientation = null, Quaternion? movingOrientation = null)
    {
        var fixedBody = PhysicsObjects.Body(mass: 0f, orientation: fixedOrientation);
        var movingBody = PhysicsObjects.Body(orientation: movingOrientation);
        var axis = Vector3.UnitX;
        var limits = new HingeAngleLimits(MinimumAngle, MaximumAngle);
        var joint = new HingeJoint(fixedBody, movingBody, Vector3.Zero, Vector3.Zero, axis, axis, limits);
        var simulation = new PhysicsSimulation(fixedBody.GameObject, movingBody.GameObject, new GameObject([joint]));
        return (movingBody, simulation);
    }

    private static void AssertAngle(float expectedAngle, RigidBody body)
    {
        var actualAngle = ReadAngle(body);
        Assert.InRange(actualAngle, expectedAngle - AngleTolerance, expectedAngle + AngleTolerance);
    }

    private static float ReadAngle(RigidBody body)
    {
        var forward = body.Transform.WorldForward;
        return MathF.Atan2(-forward.Y, forward.Z);
    }
}
