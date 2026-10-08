using System.Numerics;
using FlightEngine.Forces;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging.RigidBody;

public class GravityDebugSection : DebugSection
{
    private readonly Physics.RigidBody _body;
    private readonly Vector3 _originalGravityAcceleration;

    public GravityDebugSection(Physics.RigidBody body)
    {
        _body = body;

        var forceApplier = GetForceApplier();
        var gravity = GetGravityForce(forceApplier);

        _originalGravityAcceleration =
            gravity?.GravityAcceleration ?? Vector3.Zero;
    }

    public override void Render()
    {
        if (!RenderHeader("Gravity"))
            return;

        var forceApplier = GetForceApplier();
        if (forceApplier is null)
            return;

        RenderUseGravityEditor(forceApplier);

        var gravity = GetGravityForce(forceApplier);
        if (gravity is null)
            return;

        RenderGravityStrengthEditor(gravity);
        RenderResetGravityButton(gravity);
    }

    private ForceApplier? GetForceApplier()
    {
        return _body.GameObject.GetComponent<ForceApplier>();
    }

    private static GravityForce? GetGravityForce(ForceApplier? forceApplier)
    {
        return (GravityForce?)forceApplier?.GetForceGenerator<GravityForce>();
    }

    private static void RenderUseGravityEditor(ForceApplier forceApplier)
    {
        var useGravity = forceApplier.UseGravity;
        var useGravityChanged = ImGui.Checkbox("Use gravity", ref useGravity);
        if (useGravityChanged)
            forceApplier.UseGravity = useGravity;
    }

    private void RenderGravityStrengthEditor(GravityForce gravity)
    {
        var gravityAcceleration = gravity.GravityAcceleration;
        var gravityStrength = gravityAcceleration.Length();

        var gravityStrengthChanged = ImGui.DragFloat(
            "Gravity strength",
            ref gravityStrength,
            0.05f,
            0.0f,
            float.MaxValue,
            "%.3f",
            ImGuiSliderFlags.AlwaysClamp);

        if (!gravityStrengthChanged)
            return;

        var gravityDirection = GetGravityDirection(gravityAcceleration);
        gravity.GravityAcceleration = gravityDirection * gravityStrength;
    }

    private Vector3 GetGravityDirection(Vector3 gravityAcceleration)
    {
        if (gravityAcceleration.LengthSquared() > 0.0f)
            return Vector3.Normalize(gravityAcceleration);

        if (_originalGravityAcceleration.LengthSquared() > 0.0f)
            return Vector3.Normalize(_originalGravityAcceleration);

        return -Vector3.UnitY;
    }

    private void RenderResetGravityButton(GravityForce gravity)
    {
        if (RenderButton("Reset gravity"))
            gravity.GravityAcceleration = _originalGravityAcceleration;
    }
}