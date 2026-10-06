using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Rendering;

public class Camera(Transform target) : Component
{
    public float FovY { get; set; } = 60.0f;

    public override void OnInitialize()
    {
        var transform = GameObject.Transform;
        transform.LookAt(target.Position, Vector3.UnitY);
    }
}