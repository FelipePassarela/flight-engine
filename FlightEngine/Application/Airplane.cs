using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;
using FlightEngine.Rendering.Primitives;
using Color = Raylib_cs.Color;

namespace FlightEngine.Application;

public class Airplane
{
    public GameObject Fuselage { get; }
    public GameObject LeftWing { get; }
    public GameObject RightWing { get; }
    public IReadOnlyList<GameObject> GameObjects { get; }

    public Airplane(Vector3 position)
    {
        Fuselage = CreateFuselage(position);

        var leftRoot = Fuselage.Transform.LocalLeft * 0.3f;
        var rightRoot = Fuselage.Transform.LocalRight * 0.3f;
        LeftWing = CreateWing(leftRoot + Fuselage.Transform.LocalLeft * 1.1f, Color.Orange);
        RightWing = CreateWing(rightRoot + Fuselage.Transform.LocalRight * 1.1f, Color.SkyBlue);

        var leftJoint = CreateWingJoint(LeftWing, leftRoot);
        var rightJoint = CreateWingJoint(RightWing, rightRoot);
        GameObjects = Array.AsReadOnly<GameObject>([Fuselage, LeftWing, RightWing, leftJoint, rightJoint]);
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

    private GameObject CreateWing(Vector3 localPosition, Color color) =>
        new([
            new Transform
            {
                Position = Fuselage.Transform.TransformPoint(localPosition),
                Orientation = Fuselage.Transform.Orientation
            },
            new RigidBody(mass: 0.4f, momentOfInertia: 0.2f),
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new BoxRenderer(new Vector3(2.2f, 0.12f, 0.9f), color),
            new GizmoRenderer(axisLength: 1.0f),
        ]);

    private GameObject CreateWingJoint(GameObject wing, Vector3 localRoot) =>
        new([
            new FixedJoint(
                bodyA: Fuselage.GetComponent<RigidBody>()!,
                bodyB: wing.GetComponent<RigidBody>()!,
                contactPoint: Fuselage.Transform.TransformPoint(localRoot)),
            new FixedJointRenderer()
        ]);
}