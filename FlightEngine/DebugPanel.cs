using System.Numerics;
using FlightEngine.Rendering;
using ImGuiNET;
using Raylib_cs;
using rlImGui_cs;
using Transform = FlightEngine.Core.Transform;

namespace FlightEngine;

public class DebugPanel(Transform transform, RigidBody body, Camera camera)
{
    public void Render()
    {
        rlImGui.Begin();

        if (ImGui.Begin($"Debug - {AppInfo.Name}"))
        {
            TrackPerformance();
            ImGui.Separator();

            TrackPosition(transform);
            ImGui.Separator();

            TrackOrientation(transform);
            ImGui.Separator();

            TrackRigidBody(body);
            ImGui.Separator();

            TrackCamera(camera);
        }

        ImGui.End();
        rlImGui.End();
    }

    private static void TrackPerformance()
    {
        // TODO: Remove this dependency on Raylib
        ImGui.Text($"FPS: {Raylib.GetFPS()}");
        ImGui.Text($"Frame: {Raylib.GetFrameTime() * 1000:F2} ms");
    }

    private static void TrackPosition(Transform transform)
    {
        var position = transform.Position;
        if (ImGui.DragFloat3("Position", ref position, 0.05f))
            transform.Position = position;

        if (ImGui.Button("Reset position"))
            transform.Position = Vector3.Zero;
    }

    private static void TrackOrientation(Transform transform)
    {
        if (!ImGui.CollapsingHeader("Orientation", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        var orientation = transform.Orientation;
        ImGui.Text($"Quaternion: ({orientation.X:F4}, {orientation.Y:F4}, " +
                   $"{orientation.Z:F4}, {orientation.W:F4})");
        ImGui.Text($"Norma: {orientation.Length():F6}");

        TrackVector("World Forward", transform.WorldForward);
        TrackVector("World Right", transform.WorldRight);
        TrackVector("World Up", transform.WorldUp);

        if (ImGui.Button("Reset orientation"))
            transform.Orientation = Quaternion.Identity;
    }

    private static void TrackCamera(Camera camera)
    {
        var fov = camera.FovY;
        if (!ImGui.SliderFloat("FOV", ref fov, 20.0f, 100.0f))
            return;
        camera.FovY = fov;
    }

    private static void TrackRigidBody(RigidBody body)
    {
        TrackLinearMotion(body);
        TrackAngularMotion(body);
        ImGui.NewLine();

        if (ImGui.Button("Reset Motion"))
            body.StopMotion();

        ImGui.SameLine();

        var useGravity = body.UseGravity;
        if (ImGui.Checkbox("Use gravity", ref useGravity))
            body.UseGravity = useGravity;
    }

    private static void TrackLinearMotion(RigidBody body)
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
            TrackVector("Momentum", momentum);
            ImGui.Text($"Kinetic energy: {kineticEnergy:F3}");
        }
        else
        {
            ImGui.Text("Infinite mass (inverse mass <= 0)");
        }
    }

    private static void TrackAngularMotion(RigidBody body)
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
            TrackVector("Rotation axis", body.AngularVelocity / angularSpeed);

        var inertia = body.MomentOfInertia;
        if (ImGui.DragFloat("Moment of inertia (kg m^2)", ref inertia, 0.01f)
            && float.IsFinite(inertia) && inertia > 0.0f)
            body.MomentOfInertia = inertia;

        if (float.IsFinite(body.MomentOfInertia) && body.MomentOfInertia > 0.0f)
        {
            TrackVector("Angular momentum (kg m^2/s)",
                body.MomentOfInertia * body.AngularVelocity);
            var energy = 0.5f * body.MomentOfInertia * body.AngularVelocity.LengthSquared();
            ImGui.Text($"Rotational kinetic energy: {energy:F4} J");
        }
        else
        {
            ImGui.Text("Moment of inertia must be finite and positive.");
        }

        TrackVector("Local center of mass", body.CenterOfMass);
    }

    private static void TrackVector(string label, Vector3 value)
    {
        ImGui.Text($"{label}: ({value.X:F4}, {value.Y:F4}, {value.Z:F4})");
    }
}