using System.Numerics;
using FlightEngine.Core;
using Transform = FlightEngine.Core.Transform;

namespace FlightEngine;

public class Follow(Transform target) : Component
{
    public override void OnUpdate(float deltaTime)
    {
        var transform = GameObject.Transform;
        var displacement = -target.WorldForward * 9.0f + Vector3.UnitY * 5.0f;
        transform.Position = target.Position + displacement;
        transform.LookAt(target.Position, Vector3.UnitY);
    }
}