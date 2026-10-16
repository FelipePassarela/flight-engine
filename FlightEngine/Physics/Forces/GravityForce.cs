using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class GravityForce(Vector3 gravityAcceleration) : ForceGenerator
{
    public override Vector3 CalculateWorldForce(RigidBody body)
    {
        if (body.HasInfiniteMass)
            return Vector3.Zero;

        return body.Mass * gravityAcceleration;
    }
}