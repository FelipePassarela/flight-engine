using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Forces;
using FlightEngine.Rendering;
using FlightEngine.Rendering.Primitives;

namespace FlightEngine.Application;

public class FlightScene
{
    public GameObject Plane { get; }
    public GameObject Camera { get; }
    public GameObject Grid { get; }
    public Scene Scene { get; }

    public FlightScene()
    {
        Grid = CreateGrid();
        Plane = CreatePlane();
        Camera = CreateCamera(Plane);

        var leftWingPosition = Plane.Transform.TransformPoint(Plane.Transform.LocalLeft * 3.0f);
        var leftWing = new GameObject([
            new Transform { Position = leftWingPosition },
            new RigidBody(mass: 1.0f),
            new SphereRenderer(radius: 0.2f),
            new GizmoRenderer()
        ]);
        var leftJoint = new GameObject([
            new FixedJoint(
                bodyA: Plane.GetComponent<RigidBody>()!,
                bodyB: leftWing.GetComponent<RigidBody>()!,
                contactPoint: Plane.Transform.WorldLeft),
            new FixedJointRenderer()
        ]);

        Scene = new Scene([Plane, Grid, Camera, leftJoint, leftWing]);
    }

    private static GameObject CreatePlane() =>
        new([
            new RigidBody(mass: 1.0f),
            new FlightInputController(),
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new GizmoRenderer(),
            new SphereRenderer(radius: 0.4f),
        ]);

    private static GameObject CreateGrid() =>
        new([
            new RigidBody(mass: 0.0f),
            new GridRenderer(),
            new GizmoRenderer(axisLength: 5.0f),
            new ForceApplier()
        ]);

    private static GameObject CreateCamera(GameObject plane) =>
        new([new Camera(plane.Transform), new Follow(plane.Transform)]);
}