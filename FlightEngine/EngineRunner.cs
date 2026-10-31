using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Forces;
using FlightEngine.Rendering;
using Raylib_cs;

namespace FlightEngine;

public class EngineRunner(Scene scene, RaylibSceneRenderer sceneRenderer)
{
    private readonly PhysicsWorld _physicsWorld = new(
        scene.GetComponents<RigidBody>(),
        scene.GetComponents<ForceApplier>(),
        scene.GetComponents<Constraint>());

    public void Run()
    {
        Initialize();
        RunMainLoop();
        sceneRenderer.Close();
    }

    private void Initialize()
    {
        sceneRenderer.Initialize();
        InitializeComponents();
    }

    private void InitializeComponents()
    {
        foreach (var component in scene.GetComponents())
            component.OnInitialize();
    }

    private void RunMainLoop()
    {
        while (!Raylib.WindowShouldClose())
        {
            var deltaTime = Raylib.GetFrameTime();
            UpdateComponents(deltaTime);
            _physicsWorld.Step(deltaTime);
            sceneRenderer.Render();
        }
    }

    private void UpdateComponents(float deltaTime)
    {
        foreach (var component in scene.GetComponents())
            component.OnUpdate(deltaTime);
    }
}