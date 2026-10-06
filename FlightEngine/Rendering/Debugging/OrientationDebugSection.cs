using System.Numerics;
using FlightEngine.Core;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging;

public class OrientationDebugSection(Transform transform) : DebugSection
{
    public override void Render()
    {
        if (!ImGui.CollapsingHeader("Orientation", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        var orientation = transform.Orientation;
        ImGui.Text($"Quaternion: ({orientation.X:F4}, {orientation.Y:F4}, " +
                   $"{orientation.Z:F4}, {orientation.W:F4})");
        ImGui.Text($"Norma: {orientation.Length():F6}");

        RenderVector(transform.WorldForward, "World Forward");
        RenderVector(transform.WorldRight, "World Right");
        RenderVector(transform.WorldUp, "World Up");

        if (ImGui.Button("Reset orientation"))
            transform.Orientation = Quaternion.Identity;
    }
}