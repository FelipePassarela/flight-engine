using System.Numerics;

namespace FlightEngine.Forces;

public class ThrustForce(
    Vector3 localDirection,
    float magnitude,
    Vector3 localApplicationPoint = new())
    : ForceGenerator
{
    public override Vector3 LocalApplicationPoint => localApplicationPoint;

    public override Vector3 CalculateWorldForce(RigidBody body)
    {
        var worldDirection = Vector3.Transform(localDirection, body.Transform.Orientation);
        return worldDirection * magnitude;
    }
}