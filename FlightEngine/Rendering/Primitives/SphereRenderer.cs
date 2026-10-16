using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Rendering.Primitives;

public class SphereRenderer(float radius = 1.0f) : Renderer
{
    public override void Render()
    {
        var transform = GameObject.Transform;
        Raylib.DrawSphere(transform.Position, radius, Color.Red);
        Raylib.DrawSphereWires(transform.Position, radius, 16, 16, Color.Maroon);
    }
}