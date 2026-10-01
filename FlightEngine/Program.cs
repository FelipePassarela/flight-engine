using System.Numerics;
using FlightEngine;
using FlightEngine.Forces;
using Raylib_cs;
using Transform = FlightEngine.Transform;

var planeTransform = CreatePlaneTransform();
var planeBody = new RigidBody(planeTransform, mass: 1.0f);

var scene = new Scene(
    camera: CreateCamera(planeTransform),
    planeTransform,
    planeBody);

var sceneRenderer = new SceneRenderer(scene);
SceneRenderer.Initialize();

while (!Raylib.WindowShouldClose())
{
    FlightInputController.Update(planeBody);

    var forces = new List<ForceGenerator>
    {
        new GravityForce(GravityForce.MoonGravityAcceleration),
        new DragForce(referenceArea: 16.0f, windVelocity: Vector3.Zero)
    };

    planeBody.AddForces(forces);
    planeBody.Integrate(Raylib.GetFrameTime());

    TrackTransform(planeTransform, ref scene.Camera);

    sceneRenderer.Render();
}

SceneRenderer.Close();
return;

Transform CreatePlaneTransform() => new()
{
    Position = Vector3.Zero,
    Orientation = Quaternion.Identity,
    Scale = Vector3.One
};

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