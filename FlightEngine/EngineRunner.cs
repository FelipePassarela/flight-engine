using FlightEngine.Core;
using FlightEngine.Rendering;
using Raylib_cs;

namespace FlightEngine;

public class EngineRunner(Scene scene, RaylibSceneRenderer sceneRenderer)
{
    public void Run()
    {
        Initialize();
        RunMainLoop();
        sceneRenderer.Close();
    }

    private void Initialize()
    {
        sceneRenderer.Initialize();
        InitializeGameObjects();
    }

    private void InitializeGameObjects()
    {
        foreach (var gameObject in scene.GameObjects)
        foreach (var component in gameObject.GetComponents())
            component.OnInitialize();
    }

    private void RunMainLoop()
    {
        while (!Raylib.WindowShouldClose())
        {
            UpdateGameObjects(Raylib.GetFrameTime());
            sceneRenderer.Render();
        }
    }

    private void UpdateGameObjects(float deltaTime)
    {
        foreach (var gameObject in scene.GameObjects)
        foreach (var component in gameObject.GetComponents())
            component.OnUpdate(deltaTime);
    }
}