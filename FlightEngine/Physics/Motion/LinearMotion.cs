using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics.Motion;

internal class LinearMotion(MassProperties massProperties) : Motion
{
    private Vector3 _accumulatedForce = Vector3.Zero;
    public Vector3 Velocity { get; set; } = Vector3.Zero;

    public void AddForce(Vector3 worldForce)
    {
        _accumulatedForce += worldForce;
    }

    public void AddImpulse(Vector3 worldImpulse)
    {
        Velocity += worldImpulse * massProperties.InverseMass;
    }

    public override void IntegrateVelocity(float deltaTime)
    {
        var acceleration = _accumulatedForce * massProperties.InverseMass;
        Velocity += acceleration * deltaTime;
    }

    public override void IntegrateTransform(Transform transform, float deltaTime)
    {
        transform.Position += Velocity * deltaTime;
    }

    public override void ClearAccumulators()
    {
        _accumulatedForce = Vector3.Zero;
    }

    public override void StopMotion()
    {
        ClearAccumulators();
        Velocity = Vector3.Zero;
    }
}