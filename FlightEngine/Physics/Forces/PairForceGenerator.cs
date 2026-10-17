using System.Numerics;

namespace FlightEngine.Physics.Forces;

public abstract class PairForceGenerator : ForceGenerator
{
    public override RigidBody Body => ThisBody;
    public abstract RigidBody ThisBody { get; }
    public abstract RigidBody OtherBody { get; }
    public abstract Vector3 OtherLocalApplicationPoint { get; }

    public override Vector3 CalculateWorldForce() => CalculateWorldForceOnThis();
    public abstract Vector3 CalculateWorldForceOnThis();
}