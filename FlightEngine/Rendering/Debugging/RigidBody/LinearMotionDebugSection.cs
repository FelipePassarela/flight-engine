using ImGuiNET;

namespace FlightEngine.Rendering.Debugging.RigidBody;

public class LinearMotionDebugSection(Physics.RigidBody body) : DebugSection
{
    public override void Render()
    {
        if (!RenderHeader("Linear motion"))
            return;

        RenderInverseMassEditor();
        RenderMass();
        RenderVelocityEditor();
        RenderSpeed();
        RenderDynamicQuantities();
    }

    private void RenderInverseMassEditor()
    {
        var inverseMass = body.InverseMass;
        var inverseMassChanged = ImGui.DragFloat(
            "Inverse mass",
            v: ref inverseMass,
            v_speed: 0.01f,
            v_min: 0.0f,
            v_max: float.MaxValue,
            format: "%.3f",
            flags: ImGuiSliderFlags.AlwaysClamp);

        if (inverseMassChanged) body.InverseMass = inverseMass;
    }

    private void RenderMass()
    {
        RenderText($"Mass: {body.Mass:F3}");
    }

    private void RenderVelocityEditor()
    {
        var velocity = body.Velocity;
        var velocityChanged = ImGui.DragFloat3("Velocity", ref velocity, 0.05f);
        if (velocityChanged)
            body.Velocity = velocity;
    }

    private void RenderSpeed()
    {
        RenderText($"Speed: {body.Velocity.Length():F3}");
    }

    private void RenderDynamicQuantities()
    {
        if (body.InverseMass > 0.0f)
        {
            var momentum = body.Mass * body.Velocity;
            var kineticEnergy = 0.5f * body.Mass * body.Velocity.LengthSquared();
            RenderVector(momentum, "Momentum");
            RenderText($"Kinetic energy: {kineticEnergy:F3}");
        }
        else
        {
            RenderText("Infinite mass (inverse mass <= 0)");
        }
    }
}