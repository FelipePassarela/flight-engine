using ImGuiNET;

namespace FlightEngine.Rendering.Debugging.RigidBody;

public class AngularMotionDebugSection(Physics.RigidBody body) : DebugSection
{
    public override void Render()
    {
        if (!RenderHeader("Angular motion"))
            return;

        RenderAngularVelocityEditor();
        RenderAngularSpeed();
        RenderRotationAxis();
        RenderMomentOfInertiaEditor();
        RenderDynamicQuantities();
        RenderCenterOfMass();
    }

    private void RenderAngularVelocityEditor()
    {
        var angularVelocity = body.AngularVelocity;
        var angularVelocityChanged = ImGui.DragFloat3("Angular velocity (rad/s)", ref angularVelocity, 0.01f);
        if (angularVelocityChanged)
            body.AngularVelocity = angularVelocity;
    }

    private void RenderAngularSpeed()
    {
        var angularSpeed = body.AngularVelocity.Length();
        var angularSpeedDegrees = angularSpeed * (180.0f / MathF.PI);

        RenderText($"Angular speed: {angularSpeed:F4} rad/s");
        RenderText($"Angular speed: {angularSpeedDegrees:F2} deg/s");
    }

    private void RenderRotationAxis()
    {
        var angularSpeed = body.AngularVelocity.Length();
        if (angularSpeed > 0.0f)
            RenderVector(body.AngularVelocity / angularSpeed, "Rotation axis");
    }

    private void RenderMomentOfInertiaEditor()
    {
        var momentOfInertia = body.MomentOfInertia;
        var momentOfInertiaChanged = ImGui.DragFloat("Moment of inertia (kg m^2)", ref momentOfInertia, 0.01f);
        if (momentOfInertiaChanged && float.IsFinite(momentOfInertia) && momentOfInertia > 0.0f)
            body.MomentOfInertia = momentOfInertia;
    }

    private void RenderDynamicQuantities()
    {
        var momentOfInertia = body.MomentOfInertia;

        if (!float.IsFinite(momentOfInertia) || momentOfInertia <= 0.0f)
        {
            RenderText("Moment of inertia must be finite and positive.");
            return;
        }

        var angularMomentum = momentOfInertia * body.AngularVelocity;
        var rotationalKineticEnergy = 0.5f * momentOfInertia * body.AngularVelocity.LengthSquared();

        RenderVector(angularMomentum, "Angular momentum (kg m^2/s)");
        RenderText($"Rotational kinetic energy: {rotationalKineticEnergy:F4} J");
    }

    private void RenderCenterOfMass()
    {
        RenderVector(body.CenterOfMass, "Local center of mass");
    }
}