using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class GravityForce(RigidBody body, Vector3 gravityAcceleration) : ForceGenerator
{
    public override RigidBody Body => body;

    public override Vector3 CalculateWorldForce()
    {
        if (body.HasInfiniteMass)
            return Vector3.Zero;

        return body.Mass * gravityAcceleration;
    }
}