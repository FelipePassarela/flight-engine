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
        Scene = new Scene([Plane, Grid, Camera]);
    }

    private GameObject CreatePlane()
    {
        var gridBody = Grid.GetComponent<RigidBody>()!;
        return new GameObject([
            new RigidBody(mass: 1.0f),
            new FlightInputController(),
            new ForceApplier(),
            new Spring(otherBody: gridBody, stiffness: 10.0f, thisLocalAnchor: Vector3.UnitY),
            new Gravity(Gravity.MoonGravityAcceleration),
            new GizmoRenderer(),
            new SphereRenderer(radius: 0.4f),
            new SpringRenderer()
        ]);
    }

    private static GameObject CreateGrid() =>
        new([
            new RigidBody(mass: 0.0f),
            new GridRenderer(),
            new GizmoRenderer(axisLength: 5.0f),
        ]);

    private static GameObject CreateCamera(GameObject plane) =>
        new([new Camera(plane.Transform), new Follow(plane.Transform)]);
}