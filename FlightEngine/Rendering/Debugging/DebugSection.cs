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
}