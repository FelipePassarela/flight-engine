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
    public static (Airplane Airplane, IReadOnlyList<GameObject> GameObjects) Create(Vector3 position)
    {
        var fuselage = CreateFuselage(position);

        var leftRoot = fuselage.Transform.LocalLeft * 0.3f;
        var rightRoot = fuselage.Transform.LocalRight * 0.3f;
        var leftWing = CreateWing(fuselage, leftRoot + fuselage.Transform.LocalLeft * 1.1f, Color.Orange);
        var rightWing = CreateWing(fuselage, rightRoot + fuselage.Transform.LocalRight * 1.1f, Color.SkyBlue);

        var leftJoint = CreateWingJoint(fuselage, leftWing, leftRoot);
        var rightJoint = CreateWingJoint(fuselage, rightWing, rightRoot);
        var airplane = fuselage.AddComponent(new Airplane(leftWing, rightWing));
        var gameObjects = Array.AsReadOnly<GameObject>([fuselage, leftWing, rightWing, leftJoint, rightJoint]);

        return (airplane, gameObjects);
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
            new BoxRenderer(new Vector3(2.2f, 0.12f, 0.9f), color),
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
}