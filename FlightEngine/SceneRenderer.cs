using System.Numerics;
using Raylib_cs;
using rlImGui_cs;

namespace FlightEngine;

public class SceneRenderer(Scene scene)
{
    private Scene Scene { get; } = scene;

    public static void Initialize()
    {
        Raylib.InitWindow(1200, 800, AppInfo.Name);
        Raylib.SetTargetFPS(60);
        rlImGui.Setup();
    }

    public void Render()
    {
        Raylib.BeginDrawing();
        Raylib.ClearBackground(Color.RayWhite);

        Raylib.BeginMode3D(Scene.Camera);
        RenderGrid();
        RenderPlane(Scene);
        Raylib.EndMode3D();

        RenderUi(Scene);
        DebugPanel.Render(Scene);

        Raylib.EndDrawing();
    }

    public static void Close()
    {
        rlImGui.Shutdown();
        Raylib.CloseWindow();
    }

    private static void RenderGrid()
    {
        Raylib.DrawLine3D(Vector3.Zero, Vector3.UnitX * 5.0f, Color.Red);
        Raylib.DrawLine3D(Vector3.Zero, Vector3.UnitY * 5.0f, Color.Green);
        Raylib.DrawLine3D(Vector3.Zero, Vector3.UnitZ * 5.0f, Color.Blue);
        Raylib.DrawGrid(10, 1.0f);
    }

    private static void RenderPlane(Scene scene)
    {
        var planeTransform = scene.PlaneTransform;
        var position = planeTransform.Position;

        Raylib.DrawSphere(position + Vector3.UnitY, 1.0f, Color.Red);
        Raylib.DrawSphereWires(position + Vector3.UnitY, 1.0f, 16, 16, Color.Maroon);

        const float axisLength = 3.0f;
        Raylib.DrawLine3D(position, position + planeTransform.WorldRight * axisLength, Color.Red);
        Raylib.DrawLine3D(position, position + planeTransform.WorldUp * axisLength, Color.Green);
        Raylib.DrawLine3D(position, position + planeTransform.WorldForward * axisLength, Color.Blue);
    }

    private static void RenderUi(Scene scene)
    {
        Raylib.DrawText(AppInfo.Name, 10, 10, 20, Color.DarkGray);
        Raylib.DrawFPS(10, 40);
    }
}