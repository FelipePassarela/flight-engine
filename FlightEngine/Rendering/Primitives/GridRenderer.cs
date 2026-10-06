using Raylib_cs;

namespace FlightEngine.Rendering.Primitives;

public class GridRenderer : Renderer
{
    public override void Render()
    {
        Raylib.DrawGrid(10, 1.0f);
    }
}