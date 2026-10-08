using ImGuiNET;

namespace FlightEngine.Rendering.Debugging.RigidBody;

public class RigidBodyDebugSection(Physics.RigidBody body) : DebugSection
{
    private readonly LinearMotionDebugSection _linearMotionDebugSection = new(body);
    private readonly AngularMotionDebugSection _angularMotionDebugSection = new(body);
    private readonly GravityDebugSection _gravityDebugSection = new(body);

    public override void Render()
    {
        _linearMotionDebugSection.Render();
        _angularMotionDebugSection.Render();
        ImGui.NewLine();

        if (RenderButton("Reset Motion"))
            body.StopMotion();

        ImGui.Separator();
        _gravityDebugSection.Render();
    }
}