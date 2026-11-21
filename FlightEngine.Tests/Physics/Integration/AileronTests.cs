using System.Numerics;
using FlightEngine.Application;
using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Tests.Physics.Support;
using FlightEngine.Rendering.Primitives;

namespace FlightEngine.Physics.Tests.Physics.Integration;

[Trait("Category", "Integration")]
public class AileronTests
{
    [Fact]
    public void AirplaneFactoryMountsAnOutboardAileronBehindEachWing()
    {
        var (airplane, gameObjects) = AirplaneFactory.Create(new Vector3(2f, 3f, -4f));
        var assembly = new Scene([.. gameObjects]);
        var hinges = assembly.GetComponents<HingeJoint>().ToArray();

        Assert.Equal(2, hinges.Length);
        var leftHinge = FindWingHinge(hinges, airplane.LeftWing);
        var rightHinge = FindWingHinge(hinges, airplane.RightWing);
        Assert.NotSame(leftHinge.BodyB, rightHinge.BodyB);
        AssertAileronMount(assembly, leftHinge, airplane.LeftWing.Transform.LocalLeft);
        AssertAileronMount(assembly, rightHinge, airplane.RightWing.Transform.LocalRight);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void AileronRotatesAboutItsHingeWhileThePivotStaysAttached(bool useLeftWing)
    {
        var flightScene = new FlightScene();
        var airplane = flightScene.Airplane;
        var wing = useLeftWing ? airplane.LeftWing : airplane.RightWing;
        var hinge = FindWingHinge(flightScene.Scene.GetComponents<HingeJoint>(), wing);
        var gameObjects = flightScene.Scene.GetComponents<Transform>()
            .Select(transform => transform.GameObject)
            .ToArray();
        var simulation = new PhysicsSimulation(gameObjects);

        const float angularImpulse = 0.012f;
        var worldHingeAxis = wing.Transform.TransformDirection(hinge.LocalAxisA);
        hinge.BodyB.ApplyAngularImpulse(worldHingeAxis * angularImpulse);

        simulation.RunFor(durationSeconds: 1, stepsPerSecond: 60,
            afterStep: _ => AssertCoincidentAnchors(hinge, tolerance: 0.005f));

        var inverseWingOrientation = Quaternion.Inverse(wing.Transform.Orientation);
        var relativeAileronOrientation = inverseWingOrientation * hinge.BodyB.Transform.Orientation;
        PhysicsAssert.RotatedByAtLeast(Quaternion.Identity, relativeAileronOrientation, minimumAngleDegrees: 20f);
        AssertAlignedAxes(hinge);
    }

    private static HingeJoint FindWingHinge(IEnumerable<HingeJoint> hinges, GameObject wing) =>
        Assert.Single(hinges, hinge => hinge.BodyA.GameObject == wing);

    private static void AssertAileronMount(Scene scene, HingeJoint hinge, Vector3 outboardDirection)
    {
        var aileron = hinge.BodyB;
        Assert.Contains(aileron, scene.GetComponents<RigidBody>());
        Assert.False(aileron.HasInfiniteMass);
        Assert.NotNull(aileron.GameObject.GetComponent<BoxRenderer>());

        const float wingHalfChord = 0.45f;
        var wingTransform = hinge.BodyA.Transform;
        var distanceBehindWingCenter = -Vector3.Dot(hinge.LocalAnchorA, wingTransform.LocalForward);
        var outboardOffset = Vector3.Dot(hinge.LocalAnchorA, outboardDirection);
        var spanwiseAlignment = MathF.Abs(Vector3.Dot(hinge.LocalAxisA, outboardDirection));
        var distanceAheadOfAileronCenter = Vector3.Dot(hinge.LocalAnchorB, aileron.Transform.LocalForward);
        Assert.True(distanceBehindWingCenter >= wingHalfChord);
        Assert.True(outboardOffset > 0f);
        Assert.True(distanceAheadOfAileronCenter > 0f);
        Assert.InRange(spanwiseAlignment, 0.99999f, 1f);
        AssertCoincidentAnchors(hinge, tolerance: 1e-5f);
        AssertAlignedAxes(hinge);
    }

    private static void AssertCoincidentAnchors(HingeJoint hinge, float tolerance)
    {
        var wingAnchor = hinge.BodyA.Transform.TransformPoint(hinge.LocalAnchorA);
        var aileronAnchor = hinge.BodyB.Transform.TransformPoint(hinge.LocalAnchorB);
        PhysicsAssert.VectorNear(wingAnchor, aileronAnchor, tolerance);
    }

    private static void AssertAlignedAxes(HingeJoint hinge)
    {
        var wingAxis = hinge.BodyA.Transform.TransformDirection(hinge.LocalAxisA);
        var aileronAxis = hinge.BodyB.Transform.TransformDirection(hinge.LocalAxisB);
        PhysicsAssert.VectorNear(wingAxis, aileronAxis, tolerance: 1e-3f);
    }
}
