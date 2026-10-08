using System.Numerics;
using FlightEngine.Physics;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging;

public class RigidBodyDebugSection(RigidBody body) : DebugSection
{
    public override void Render()
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
        var dragFloat = ImGui.DragFloat(
            "Inverse mass",
            v: ref inverseMass,
            v_speed: 0.01f,
            v_min: 0.0f,
            v_max: float.MaxValue,
            format: "%.3f",
            flags: ImGuiSliderFlags.AlwaysClamp);
        if (dragFloat) body.InverseMass = inverseMass;
        
        ImGui.Text($"Mass: {body.Mass:F3}");

        var velocity = body.Velocity;
        if (ImGui.DragFloat3("Velocity", ref velocity, 0.05f))
            body.Velocity = velocity;

        ImGui.Text($"Speed: {body.Velocity.Length():F3}");

        if (body.InverseMass > 0.0f)
        {
            var momentum = body.Mass * body.Velocity;
            var kineticEnergy = 0.5f * body.Mass * body.Velocity.LengthSquared();
            RenderVector(momentum, "Momentum");
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
            RenderVector(body.AngularVelocity / angularSpeed, "Rotation axis");

        var inertia = body.MomentOfInertia;
        if (ImGui.DragFloat("Moment of inertia (kg m^2)", ref inertia, 0.01f)
            && float.IsFinite(inertia) && inertia > 0.0f)
            body.MomentOfInertia = inertia;

        if (float.IsFinite(body.MomentOfInertia) && body.MomentOfInertia > 0.0f)
        {
            var angularMomentum = body.MomentOfInertia * body.AngularVelocity;
            RenderVector(angularMomentum, "Angular momentum (kg m^2/s)");
            var energy = 0.5f * body.MomentOfInertia * body.AngularVelocity.LengthSquared();
            ImGui.Text($"Rotational kinetic energy: {energy:F4} J");
        }
        else
        {
            ImGui.Text("Moment of inertia must be finite and positive.");
        }

        RenderVector(body.CenterOfMass, "Local center of mass");
    }
}