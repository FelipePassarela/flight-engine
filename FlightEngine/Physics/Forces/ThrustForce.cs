using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class ThrustForce(
    RigidBody body,
    Vector3 localDirection,
    float magnitude,
    Vector3 localApplicationPoint = new())
    : ForceGenerator
{
    public override RigidBody Body => body;
    public override Vector3 LocalApplicationPoint => localApplicationPoint;

    public override Vector3 CalculateWorldForce()
    {
        var worldDirection = Vector3.Transform(localDirection, body.Transform.Orientation);
        return worldDirection * magnitude;
    }
}