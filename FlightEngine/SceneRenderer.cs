using Raylib_cs;
using rlImGui_cs;
using FlightEngine.Rendering;

namespace FlightEngine;

public class SceneRenderer(Scene scene, DebugPanel? debugPanel)
{
    private Scene Scene { get; } = scene;

    public static void Initialize()
    {
        Raylib.InitWindow(1600, 900, AppInfo.Name);
        Raylib.SetTargetFPS(60);
        rlImGui.Setup();
    }

    public void Render()
    {
        Raylib.BeginDrawing();
        Raylib.ClearBackground(Color.RayWhite);

        Raylib.BeginMode3D(Scene.Camera);
        RenderComponents();
        Raylib.EndMode3D();

        RenderUi();

        Raylib.EndDrawing();
    }

    private void RenderComponents()
    {
        foreach (var gameObject in Scene.GameObjects)
        foreach (var renderer in gameObject.GetComponentsOfType<Renderer>())
            renderer.Render();
    }

    private void RenderUi()
    {
        Raylib.DrawText(AppInfo.Name, 10, 10, 20, Color.DarkGray);
        Raylib.DrawFPS(10, 40);
        debugPanel?.Render();
    }

    public static void Close()
    {
        rlImGui.Shutdown();
        Raylib.CloseWindow();
    }
}