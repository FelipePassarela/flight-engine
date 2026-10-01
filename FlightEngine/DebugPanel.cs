using System.Numerics;
using ImGuiNET;
using Raylib_cs;
using rlImGui_cs;

namespace FlightEngine;

public static class DebugPanel
{
    public static void Render(Scene scene)
    {
        rlImGui.Begin();

        if (ImGui.Begin($"Debug - {AppInfo.Name}"))
        {
            RenderPerformance();
            ImGui.Separator();

            RenderPosition(scene.PlaneTransform);
            ImGui.Separator();

            RenderOrientation(scene.PlaneTransform);
            ImGui.Separator();

            RenderRigidBody(scene.PlaneBody);
            ImGui.Separator();

            RenderCamera(scene);
        }

        ImGui.End();
        rlImGui.End();
    }

    private static void RenderPerformance()
    {
        ImGui.Text($"FPS: {Raylib.GetFPS()}");
        ImGui.Text($"Frame: {Raylib.GetFrameTime() * 1000:F2} ms");
    }

    private static void RenderPosition(Transform transform)
    {
        var position = transform.Position;
        if (ImGui.DragFloat3("Position", ref position, 0.05f))
            transform.Position = position;

        if (ImGui.Button("Reset position"))
            transform.Position = Vector3.Zero;
    }

    private static void RenderOrientation(Transform transform)
    {
        if (!ImGui.CollapsingHeader("Orientation", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        var orientation = transform.Orientation;
        ImGui.Text($"Quaternion: ({orientation.X:F4}, {orientation.Y:F4}, " +
                   $"{orientation.Z:F4}, {orientation.W:F4})");
        ImGui.Text($"Norma: {orientation.Length():F6}");
        ImGui.Text($"World Forward: {transform.WorldForward}");
        ImGui.Text($"World Right: {transform.WorldRight}");
        ImGui.Text($"World Up: {transform.WorldUp}");

        if (ImGui.Button("Reset orientation"))
            transform.Orientation = Quaternion.Identity;
    }

    private static void RenderCamera(Scene scene)
    {
        var camera = scene.Camera;
        var fov = camera.FovY;
        if (!ImGui.SliderFloat("FOV", ref fov, 20.0f, 100.0f))
            return;

        camera.FovY = fov;
        scene.Camera = camera;
    }

    private static void RenderRigidBody(RigidBody body)
    {
        if (!ImGui.CollapsingHeader("RigidBody", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        var inverseMass = body.InverseMass;
        if (ImGui.DragFloat("Inverse mass", ref inverseMass, 0.01f))
            body.InverseMass = inverseMass;
        ImGui.Text($"Mass: {body.Mass:F3}");

        var velocity = body.Velocity;
        if (ImGui.DragFloat3("Velocity", ref velocity, 0.05f))
            body.Velocity = velocity;

        ImGui.Text($"Speed: {body.Velocity.Length():F3}");

        if (body.InverseMass > 0.0f)
        {
            var momentum = body.Mass * body.Velocity;
            var kineticEnergy = 0.5f * body.Mass * body.Velocity.LengthSquared();
            ImGui.Text($"Momentum: ({momentum.X:F3}, {momentum.Y:F3}, {momentum.Z:F3})");
            ImGui.Text($"Kinetic energy: {kineticEnergy:F3}");
        }
        else
        {
            ImGui.Text("Infinite mass (inverse mass <= 0)");
        }

        if (ImGui.Button("Stop body"))
            body.StopMotion();
    }
}