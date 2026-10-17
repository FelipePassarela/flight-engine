using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class DragForce(
    RigidBody body,
    float referenceArea,
    float dragCoefficient = 0.05f,
    float airDensity = 1.225f,
    Vector3 windVelocity = new())
    : ForceGenerator
{
    public override RigidBody Body => body;

    public override Vector3 CalculateWorldForce()
    {
        var speed = body.Velocity.Length();
        var relativeVelocity = body.Velocity - windVelocity;
        return -0.5f * airDensity * dragCoefficient * referenceArea * speed * relativeVelocity;
    }
}