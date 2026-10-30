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

        var spring = new GameObject([
            new Spring(
                bodyA: Plane.GetComponent<RigidBody>()!,
                bodyB: Grid.GetComponent<RigidBody>()!,
                localAnchorA: Vector3.UnitY,
                stiffness: 10.0f),
            new SpringRenderer()
        ]);

        Scene = new Scene([Plane, Grid, Camera, spring]);
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