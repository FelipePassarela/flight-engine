using ImGuiNET;

namespace FlightEngine.Rendering.Debugging;

public class CameraDebugSection(Camera camera) : DebugSection
{
    public override void Render()
    {
        var fov = camera.FovY;
        if (!ImGui.SliderFloat("FOV", ref fov, 20.0f, 100.0f))
            return;
        camera.FovY = fov;
    }
}