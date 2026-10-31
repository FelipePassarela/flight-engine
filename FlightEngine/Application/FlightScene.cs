using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Forces;
using FlightEngine.Rendering;
using FlightEngine.Rendering.Primitives;

namespace FlightEngine.Application;

public class FlightScene
{
    public Airplane Airplane { get; }
    public GameObject Camera { get; }
    public GameObject Grid { get; }
    public Scene Scene { get; }

    public FlightScene()
    {
        Grid = CreateGrid();
        Airplane = new Airplane(new Vector3(0.0f, 3.0f, 0.0f));
        Camera = CreateCamera(Airplane.Fuselage);
        Scene = new Scene([.. Airplane.GameObjects, Grid, Camera]);
    }

    private static GameObject CreateGrid() =>
        new([
            new RigidBody(mass: 0.0f),
            new GridRenderer(),
            new GizmoRenderer(axisLength: 5.0f),
            new ForceApplier()
        ]);

    private static GameObject CreateCamera(GameObject airplane) =>
        new([
            new Transform { Position = airplane.Transform.TransformPoint(new Vector3(0.0f, 5.0f, -9.0f)) },
            new Camera(airplane.Transform),
            new Follow(airplane.Transform)
        ]);
}