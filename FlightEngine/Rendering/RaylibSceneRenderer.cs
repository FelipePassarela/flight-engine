using FlightEngine.Core;
using FlightEngine.Rendering.Debugging;
using FlightEngine.Rendering.Primitives;
using Raylib_cs;
using rlImGui_cs;

namespace FlightEngine.Rendering;

public class RaylibSceneRenderer(
    Scene scene,
    Camera activeCamera,
    DebugPanel? debugPanel,
    string appName = "App")
{
    public void Initialize()
    {
        Raylib.InitWindow(1600, 900, appName);
        Raylib.SetTargetFPS(60);
        rlImGui.Setup();
    }

    public void Render()
    {
        Raylib.BeginDrawing();
        Raylib.ClearBackground(Color.RayWhite);

        Raylib.BeginMode3D(CreateRaylibCamera(activeCamera));
        RenderComponents();
        Raylib.EndMode3D();

        RenderUi();

        Raylib.EndDrawing();
    }

    private void RenderComponents()
    {
        foreach (var renderer in scene.GetComponents<Renderer>())
            renderer.Render();
    }

    private void RenderUi()
    {
        Raylib.DrawText(appName, 10, 10, 20, Color.DarkGray);
        Raylib.DrawFPS(10, 40);
        debugPanel?.Render();
    }

    public void Close()
    {
        rlImGui.Shutdown();
        Raylib.CloseWindow();
    }

    private static Camera3D CreateRaylibCamera(Camera camera)
    {
        var transform = camera.GameObject.Transform;
        return new Camera3D
        {
            Position = transform.Position,
            Target = transform.Position + transform.WorldForward,
            Up = transform.WorldUp,
            FovY = camera.FovY,
            Projection = CameraProjection.Perspective
        };
    }
}