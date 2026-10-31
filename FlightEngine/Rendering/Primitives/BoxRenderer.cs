using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Rendering.Primitives;

public class BoxRenderer(Vector3 size, Color color) : Renderer
{
    private const float RotationThreshold = 1e-6f;

    public override void Render()
    {
        var transform = GameObject.Transform;
        var dimensions = size * transform.Scale;

        Rlgl.PushMatrix();
        Rlgl.Translatef(transform.Position.X, transform.Position.Y, transform.Position.Z);
        ApplyRotation(transform.Orientation);

        Raylib.DrawCubeV(Vector3.Zero, dimensions, color);
        Raylib.DrawCubeWiresV(Vector3.Zero, dimensions, Color.DarkGray);
        
        Rlgl.PopMatrix();
    }

    private static void ApplyRotation(Quaternion orientation)
    {
        orientation = Quaternion.Normalize(orientation);
        var axis = new Vector3(orientation.X, orientation.Y, orientation.Z);
        var axisLength = axis.Length();
        if (axisLength <= RotationThreshold) return;

        axis /= axisLength;
        var angleRadians = 2.0f * MathF.Atan2(axisLength, orientation.W);
        var angleDegrees = angleRadians * (180.0f / MathF.PI);
        Rlgl.Rotatef(angleDegrees, axis.X, axis.Y, axis.Z);
    }
}
