using ImGuiNET;
using Raylib_cs;

namespace FlightEngine.Rendering.Debugging;

public class PerformanceDebugSection : DebugSection
{
    public override void Render()
    {
        // TODO: Remove this dependency on Raylib
        ImGui.Text($"FPS: {Raylib.GetFPS()}");
        ImGui.SameLine();
        ImGui.Text($"Frame: {Raylib.GetFrameTime() * 1000:F2} ms");
    }
}