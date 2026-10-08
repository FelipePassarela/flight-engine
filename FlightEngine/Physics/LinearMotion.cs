using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

internal class LinearMotion(MassProperties massProperties) : IMotion
{
    private Vector3 _accumulatedForce = Vector3.Zero;
    public Vector3 Velocity { get; set; } = Vector3.Zero;

    public void Integrate(Transform transform, float deltaTime)
    {
        var acceleration = _accumulatedForce * massProperties.InverseMass;
        Velocity += acceleration * deltaTime;
        transform.Position += Velocity * deltaTime;
        _accumulatedForce = Vector3.Zero;
    }

    public void AddForce(Vector3 worldForce)
    {
        _accumulatedForce += worldForce;
    }

    public void StopMotion()
    {
        _accumulatedForce = Vector3.Zero;
        Velocity = Vector3.Zero;
    }
}