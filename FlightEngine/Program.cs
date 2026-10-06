using System.Numerics;
using FlightEngine;
using FlightEngine.Core;
using FlightEngine.Forces;
using FlightEngine.Rendering;
using Raylib_cs;
using Transform = FlightEngine.Core.Transform;

// TODO: Extract main loop and entities into a FlightEngine (or better name) class
// TODO: Consider refactoring to use LifeCycle Callbacks pattern for components

var plane = new GameObject(
    [
        new RigidBody(mass: 1.0f),
        new FlightInputController(),
        new GizmoRenderer(),
        new SphereRenderer()
    ]
);
var grid = new GameObject(
    [
        new GridRenderer(),
        new GizmoRenderer(axisLength: 5.0f)
    ]
);
var scene = new Scene(camera: CreateCamera(plane.Transform), [plane, grid]);

var planeBody = plane.GetComponent<RigidBody>()!;
var debugPanel = new DebugPanel(plane.Transform, planeBody, scene.Camera);
var sceneRenderer = new SceneRenderer(scene, debugPanel);

SceneRenderer.Initialize();

plane.GetComponent<FlightInputController>()?.OnInitialize();

while (!Raylib.WindowShouldClose())
{
    plane.GetComponent<FlightInputController>()?.Update();

    var forces = new List<ForceGenerator>
    {
        new GravityForce(GravityForce.MoonGravityAcceleration),
        new DragForce(referenceArea: 16.0f, windVelocity: Vector3.Zero)
    };

    planeBody.AddForces(forces);
    planeBody.Integrate(Raylib.GetFrameTime());

    TrackTransform(plane.Transform, ref scene.Camera);

    sceneRenderer.Render();
}

SceneRenderer.Close();
return;

Camera3D CreateCamera(Transform planeTransform)
{
    var cameraDisplacement = Vector3.One * 5.0f;
    return new Camera3D
    {
        Position = planeTransform.Position + cameraDisplacement,
        Target = planeTransform.Position,
        Up = planeTransform.WorldUp,
        FovY = 60.0f,
        Projection = CameraProjection.Perspective
    };
}

void TrackTransform(Transform transform, ref Camera3D camera)
{
    var cameraDisplacement = -transform.WorldForward * 9.0f + Vector3.UnitY * 5.0f;
    camera.Position = transform.Position + cameraDisplacement;
    camera.Target = transform.Position;
    camera.Up = transform.WorldUp;
}