using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;
using FlightEngine.Rendering;
using FlightEngine.Rendering.Primitives;
using Color = Raylib_cs.Color;

namespace FlightEngine.Application;

public static class AirplaneFactory
{
    private static readonly Vector3 WingSize = new(2.2f, 0.12f, 0.9f);
    private static readonly Vector3 AileronSize = new(1.0f, 0.08f, 0.28f);
    private const float AileronMaximumDeflection = 25f * (MathF.PI / 180f);
    private static readonly HingeAngleLimits AileronLimits = new(-AileronMaximumDeflection, AileronMaximumDeflection);

    public static (Airplane Airplane, IReadOnlyList<GameObject> GameObjects) Create(Vector3 position)
    {
        var fuselage = CreateFuselage(position);
        var leftWing = CreateWingAssembly(fuselage, fuselage.Transform.LocalLeft, Color.Orange, Color.Brown);
        var rightWing = CreateWingAssembly(fuselage, fuselage.Transform.LocalRight, Color.SkyBlue, Color.Blue);
        var airplane = fuselage.AddComponent(new Airplane(leftWing.Wing, rightWing.Wing));

        var gameObjects = Array.AsReadOnly<GameObject>([
            fuselage,
            leftWing.Wing,
            rightWing.Wing,
            leftWing.WingJoint,
            rightWing.WingJoint,
            leftWing.Aileron,
            leftWing.AileronHinge,
            rightWing.Aileron,
            rightWing.AileronHinge
        ]);

        return (airplane, gameObjects);
    }

    private static (GameObject Wing, GameObject Aileron, GameObject WingJoint, GameObject AileronHinge)
        CreateWingAssembly(GameObject fuselage, Vector3 outboardDirection, Color wingColor, Color aileronColor)
    {
        const float fuselageHalfWidth = 0.3f;
        var wingRoot = outboardDirection * fuselageHalfWidth;
        var halfWingSpan = WingSize.X * 0.5f;
        var wingCenter = wingRoot + outboardDirection * halfWingSpan;
        var wing = CreateWing(fuselage, wingCenter, wingColor);
        var wingJoint = CreateWingJoint(fuselage, wing, wingRoot);
        var (aileron, aileronHinge) = CreateWingAileron(wing, outboardDirection, aileronColor);
        return (wing, aileron, wingJoint, aileronHinge);
    }

    private static GameObject CreateFuselage(Vector3 position) =>
        new([
            new Transform { Position = position },
            new RigidBody(mass: 2.0f, momentOfInertia: 1.5f),
            new FlightInputController(),
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new GizmoRenderer(axisLength: 2.0f),
            new BoxRenderer(new Vector3(0.6f, 0.6f, 3.0f), Color.LightGray),
        ]);

    private static GameObject CreateWing(GameObject fuselage, Vector3 localPosition, Color color) =>
        new([
            new Transform
            {
                Position = fuselage.Transform.TransformPoint(localPosition),
                Orientation = fuselage.Transform.Orientation
            },
            new RigidBody(mass: 0.4f, momentOfInertia: 0.2f),
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new BoxRenderer(WingSize, color),
            new GizmoRenderer(axisLength: 1.0f),
        ]);

    private static GameObject CreateWingJoint(GameObject fuselage, GameObject wing, Vector3 localRoot) =>
        new([
            new FixedJoint(
                bodyA: fuselage.GetComponent<RigidBody>()!,
                bodyB: wing.GetComponent<RigidBody>()!,
                contactPoint: fuselage.Transform.TransformPoint(localRoot)),
            new FixedJointRenderer()
        ]);

    private static (GameObject Aileron, GameObject Hinge) CreateWingAileron(
        GameObject wing, Vector3 outboardDirection, Color color)
    {
        var wingAnchor = CalculateAileronMount(wing.Transform, outboardDirection);
        var aileronAnchor = wing.Transform.LocalForward * (AileronSize.Z * 0.5f);
        var aileron = CreateAileron(wing.Transform, wingAnchor, aileronAnchor, color);
        var hinge = CreateAileronHinge(wing, aileron, wingAnchor, aileronAnchor);
        return (aileron, hinge);
    }

    private static Vector3 CalculateAileronMount(Transform wing, Vector3 outboardDirection)
    {
        const float clearance = 0.02f;
        var outerHalfCenter = WingSize.X * 0.25f;
        var halfChord = WingSize.Z * 0.5f;
        return outboardDirection * outerHalfCenter - wing.LocalForward * (halfChord + clearance);
    }

    private static GameObject CreateAileron(Transform wing, Vector3 wingAnchor, Vector3 aileronAnchor, Color color)
    {
        var hingePosition = wing.TransformPoint(wingAnchor);
        var aileronAnchorOffset = wing.TransformDirection(aileronAnchor);
        return new GameObject([
            new Transform
            {
                Position = hingePosition - aileronAnchorOffset,
                Orientation = wing.Orientation
            },
            new RigidBody(mass: 0.05f, momentOfInertia: 0.01f),
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new BoxRenderer(AileronSize, color)
        ]);
    }

    private static GameObject CreateAileronHinge(
        GameObject wing, GameObject aileron, Vector3 wingAnchor, Vector3 aileronAnchor) =>
        new([
            new HingeJoint(
                bodyA: wing.GetComponent<RigidBody>()!,
                bodyB: aileron.GetComponent<RigidBody>()!,
                localAnchorA: wingAnchor,
                localAnchorB: aileronAnchor,
                localAxisA: wing.Transform.LocalLeft,
                localAxisB: aileron.Transform.LocalLeft,
                angleLimits: AileronLimits)
        ]);
}
