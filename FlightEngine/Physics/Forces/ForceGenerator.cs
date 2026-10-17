using System.Numerics;

namespace FlightEngine.Physics.Forces;

public abstract class ForceGenerator
{
    public abstract RigidBody Body { get; }

    public virtual Vector3 LocalApplicationPoint => Vector3.Zero;
    
    public abstract Vector3 CalculateWorldForce();
}