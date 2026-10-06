using ImGuiNET;
using rlImGui_cs;

namespace FlightEngine.Rendering.Debugging;

// TODO: Make this a component and attach it to a game object.
public class DebugPanel(IReadOnlyList<DebugSection> sections, string appName = "App")
{
    public void Render()
    {
        rlImGui.Begin();

        if (ImGui.Begin($"Debug - {appName}"))
        {
            RenderSections();
        }

        ImGui.End();
        rlImGui.End();
    }

    private void RenderSections()
    {
        for (var i = 0; i < sections.Count; i++)
        {
            if (i > 0) ImGui.Separator();
            sections[i].Render();
        }
    }
}