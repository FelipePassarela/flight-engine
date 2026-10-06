using System.Numerics;
using FlightEngine;
using FlightEngine.Core;
using FlightEngine.Forces;
using FlightEngine.Rendering;
using Raylib_cs;

// TODO: Extract main loop and entities into a FlightApplication (or better name) class

var plane = new GameObject([
    new RigidBody(mass: 1.0f),
    new FlightInputController(),
    new ForceApplier([
        new GravityForce(GravityForce.MoonGravityAcceleration),
        new DragForce(referenceArea: 16.0f, windVelocity: Vector3.Zero),
    ]),
    new GizmoRenderer(),
    new SphereRenderer(),
]);

var grid = new GameObject(
    [new GridRenderer(), new GizmoRenderer(axisLength: 5.0f)]);

var camera = new GameObject(
    [new Camera(plane.Transform), new Follow(plane.Transform)]);

var activeCamera = camera.GetComponent<Camera>()!;
var scene = new Scene(activeCamera: activeCamera, [plane, grid, camera]);

var planeBody = plane.GetComponent<RigidBody>()!;
var debugPanel = new DebugPanel(plane.Transform, planeBody, scene.ActiveCamera);
var sceneRenderer = new SceneRenderer(scene, debugPanel);

SceneRenderer.Initialize();

foreach (var gameObject in scene.GameObjects)
foreach (var component in gameObject.GetComponents())
    component.OnInitialize();

while (!Raylib.WindowShouldClose())
{
    var deltaTime = Raylib.GetFrameTime();

    foreach (var gameObject in scene.GameObjects)
    foreach (var component in gameObject.GetComponents())
        component.OnUpdate(deltaTime);

    sceneRenderer.Render();
}

SceneRenderer.Close();