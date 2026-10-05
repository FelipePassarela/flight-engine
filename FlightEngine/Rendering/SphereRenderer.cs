using System.Numerics;
using Raylib_cs;

namespace FlightEngine.Rendering;

public class SphereRenderer : Renderer
{
    public override void Render()
    {
        var transform = GameObject.Transform;
        Raylib.DrawSphere(transform.Position + Vector3.UnitY, 1.0f, Color.Red);
        Raylib.DrawSphereWires(transform.Position + Vector3.UnitY, 1.0f, 16, 16, Color.Maroon);
    }
}