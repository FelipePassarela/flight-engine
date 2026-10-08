using System.Numerics;
using FlightEngine.Core;
using ImGuiNET;

namespace FlightEngine.Rendering.Debugging;

public class PositionDebugSection(Transform transform) : DebugSection
{
    public override void Render()
    {
        var position = transform.Position;
        var positionChanged = ImGui.DragFloat3("Position", ref position, 0.05f);
        if (positionChanged)
            transform.Position = position;

        if (RenderButton("Reset position"))
            transform.Position = Vector3.Zero;
    }
}