using System.Numerics;
using FlightEngine.Physics;

namespace FlightEngine.Forces;

public class DragForce(
    float referenceArea,
    float dragCoefficient = 0.05f,
    float airDensity = 1.225f,
    Vector3 windVelocity = new())
    : ForceGenerator
{
    public override Vector3 CalculateWorldForce(RigidBody body)
    {
        var speed = body.Velocity.Length();
        var relativeVelocity = body.Velocity - windVelocity;
        return -0.5f * airDensity * dragCoefficient * referenceArea * speed * relativeVelocity;
    }
}