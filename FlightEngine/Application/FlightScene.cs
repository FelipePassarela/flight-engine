using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Constraints;
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
        var (airplane, airplaneGameObjects) = AirplaneFactory.Create(new Vector3(0.0f, 3.0f, 0.0f));
        Airplane = airplane;
        Grid = CreateGrid();
        Camera = CreateCamera(Airplane.Fuselage);
        var tetheredPayload = CreateTetheredPayload(Airplane.Fuselage);
        Scene = new Scene([.. airplaneGameObjects, .. tetheredPayload, Grid, Camera]);
    }

    private static GameObject[] CreateTetheredPayload(GameObject fuselage)
    {
        const float tetherLength = 2.0f;
        var fuselageAnchor = Vector3.Zero;
        var initialDirection = new Vector3(0.6f, -0.8f, 0.0f);
        var payloadLocalPosition = fuselageAnchor + initialDirection * tetherLength;
        var payloadPosition = fuselage.Transform.TransformPoint(payloadLocalPosition);
        var payloadVelocity = -fuselage.Transform.WorldForward * 1.5f;
        var payload = CreatePayload(payloadPosition, payloadVelocity);

        var tether = new GameObject([
            new DistanceJoint(
                bodyA: fuselage.GetComponent<RigidBody>()!,
                bodyB: payload.GetComponent<RigidBody>()!,
                localAnchorA: fuselageAnchor,
                localAnchorB: Vector3.Zero,
                targetDistance: tetherLength),
            new DistanceJointRenderer()
        ]);

        return [payload, tether];
    }

    private static GameObject CreatePayload(Vector3 position, Vector3 velocity) =>
        new([
            new Transform { Position = position },
            new RigidBody(mass: 0.2f) { Velocity = velocity },
            new ForceApplier(),
            new Gravity(Gravity.MoonGravityAcceleration),
            new SphereRenderer(radius: 0.3f)
        ]);

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
