using System.Numerics;
using FlightEngine.Physics;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging.RigidBody;

public class GravityDebugSection : DebugSection
{
    private readonly Physics.RigidBody _body;
    private readonly Vector3 _originalGravityAcceleration;

    public GravityDebugSection(Physics.RigidBody body)
    {
        _body = body;
        _originalGravityAcceleration = GetGravityComponent()?.GravityAcceleration ?? Vector3.Zero;
    }

    public override void Render()
    {
        if (!RenderHeader("Gravity"))
            return;

        var gravity = GetGravityComponent();
        if (gravity is null)
            return;

        RenderUseGravityEditor(gravity);
        RenderGravityStrengthEditor(gravity);
        RenderResetGravityButton(gravity);
    }

    private Gravity? GetGravityComponent()
    {
        return _body.GameObject.GetComponent<Gravity>();
    }

    private static void RenderUseGravityEditor(Gravity gravity)
    {
        var useGravity = gravity.Active;
        var useGravityChanged = ImGui.Checkbox("Use gravity", ref useGravity);
        if (useGravityChanged)
            gravity.Active = useGravity;
    }

    private void RenderGravityStrengthEditor(Gravity gravity)
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

    private void RenderResetGravityButton(Gravity gravity)
    {
        if (RenderButton("Reset gravity"))
            gravity.GravityAcceleration = _originalGravityAcceleration;
    }
}