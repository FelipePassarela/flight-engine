using Raylib_cs;

namespace FlightEngine.Rendering;

public class GizmoRenderer(float axisLength = 3.0f) : Renderer
{
    public override void Render()
    {
        // TODO: Invert these Raylib dependency for the entire Rendering module.
        var transform = GameObject.Transform;
        var position = transform.Position;

        var rightAxis = position + transform.WorldRight * axisLength;
        var upAxis = position + transform.WorldUp * axisLength;
        var forwardAxis = position + transform.WorldForward * axisLength;

        Raylib.DrawLine3D(position, rightAxis, Color.Red);
        Raylib.DrawLine3D(position, upAxis, Color.Green);
        Raylib.DrawLine3D(position, forwardAxis, Color.Blue);
    }
}