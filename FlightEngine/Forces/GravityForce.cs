using System.Numerics;
using FlightEngine.Physics;

namespace FlightEngine.Forces;

public class GravityForce(Vector3 gravityAcceleration) : ForceGenerator
{
    public static Vector3 EarthGravityAcceleration => new(0.0f, -9.81f, 0.0f);
    public static Vector3 MoonGravityAcceleration => new(0.0f, -1.62f, 0.0f);

    public override Vector3 CalculateWorldForce(RigidBody body)
    {
        if (body.HasInfiniteMass)
            return Vector3.Zero;

        return body.Mass * gravityAcceleration;
    }
}