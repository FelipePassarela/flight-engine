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

        RenderVector("World Forward", transform.WorldForward);
        RenderVector("World Right", transform.WorldRight);
        RenderVector("World Up", transform.WorldUp);

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
        RenderLinearMotion(body);
        RenderAngularMotion(body);
        ImGui.NewLine();
        
        if (ImGui.Button("Reset Motion"))
            body.StopMotion();
    }

    private static void RenderLinearMotion(RigidBody body)
    {
        if (!ImGui.CollapsingHeader("Linear motion", ImGuiTreeNodeFlags.DefaultOpen))
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
            RenderVector("Momentum", momentum);
            ImGui.Text($"Kinetic energy: {kineticEnergy:F3}");
        }
        else
        {
            ImGui.Text("Infinite mass (inverse mass <= 0)");
        }
    }

    private static void RenderAngularMotion(RigidBody body)
    {
        if (!ImGui.CollapsingHeader("Angular motion", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        var angularVelocity = body.AngularVelocity;
        if (ImGui.DragFloat3("Angular velocity (rad/s)", ref angularVelocity, 0.01f))
            body.AngularVelocity = angularVelocity;

        var angularSpeed = body.AngularVelocity.Length();
        ImGui.Text($"Angular speed: {angularSpeed:F4} rad/s");
        ImGui.Text($"Angular speed: {angularSpeed * (180.0f / MathF.PI):F2} deg/s");
        if (angularSpeed > 0.0f)
            RenderVector("Rotation axis", body.AngularVelocity / angularSpeed);

        var inertia = body.MomentOfInertia;
        if (ImGui.DragFloat("Moment of inertia (kg m^2)", ref inertia, 0.01f)
            && float.IsFinite(inertia) && inertia > 0.0f)
            body.MomentOfInertia = inertia;

        if (float.IsFinite(body.MomentOfInertia) && body.MomentOfInertia > 0.0f)
        {
            RenderVector("Angular momentum (kg m^2/s)",
                body.MomentOfInertia * body.AngularVelocity);
            var energy = 0.5f * body.MomentOfInertia * body.AngularVelocity.LengthSquared();
            ImGui.Text($"Rotational kinetic energy: {energy:F4} J");
        }
        else
        {
            ImGui.Text("Moment of inertia must be finite and positive.");
        }

        RenderVector("Local center of mass", body.CenterOfMass);
    }

    private static void RenderVector(string label, Vector3 value)
    {
        ImGui.Text($"{label}: ({value.X:F4}, {value.Y:F4}, {value.Z:F4})");
    }
}