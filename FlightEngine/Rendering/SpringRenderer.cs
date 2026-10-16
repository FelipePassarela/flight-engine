using FlightEngine.Physics;
using Raylib_cs;

namespace FlightEngine.Rendering;

internal class SpringRenderer : Renderer
{
    private Spring _spring = null!;

    public override void OnInitialize()
    {
        _spring = GameObject.GetComponent<Spring>() ??
                  throw new InvalidOperationException("SpringRenderer requires a Spring component.");
    }

    public override void Render()
    {
        Raylib.DrawSphere(_spring.ThisWorldAnchor, 0.2f, Color.DarkBlue);
        Raylib.DrawSphere(_spring.OtherWorldAnchor, 0.2f, Color.DarkBlue);
        Raylib.DrawLine3D(_spring.ThisWorldAnchor, _spring.OtherWorldAnchor, Color.DarkBlue);
    }
}