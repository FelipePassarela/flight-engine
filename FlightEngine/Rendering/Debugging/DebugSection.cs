using System.Numerics;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging;

public abstract class DebugSection
{
    public abstract void Render();

    protected static void RenderVector(Vector3 value, string label)
    {
        ImGui.Text($"{label}: ({value.X:F4}, {value.Y:F4}, {value.Z:F4})");
    }

    protected static void RenderText(string fmt)
    {
        ImGui.Text(fmt);
    }

    protected static bool RenderHeader(string label)
    {
        return ImGui.CollapsingHeader(label, ImGuiTreeNodeFlags.DefaultOpen);
    }

    protected static bool RenderButton(string label)
    {
        return ImGui.Button(label);
    }
}