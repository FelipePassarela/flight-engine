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
    public Scene Scene { get; }

    public FlightScene()
    {
        Plane = CreatePlane();
        Camera = CreateCamera(Plane);
        Scene = new Scene([Plane, CreateGrid(), Camera]);
    }

    private static GameObject CreatePlane() =>
        new([
            new RigidBody(mass: 1.0f),
            new FlightInputController(),
            new ForceApplier(),
            new Spring(stiffness: 10.0f, localAnchor: Vector3.UnitY),
            new GizmoRenderer(),
            new SphereRenderer(radius: 0.4f),
            new SpringRenderer()
        ]);

    private static GameObject CreateGrid() => new([new GridRenderer(), new GizmoRenderer(axisLength: 5.0f)]);

    private static GameObject CreateCamera(GameObject plane) =>
        new([new Camera(plane.Transform), new Follow(plane.Transform)]);
}
