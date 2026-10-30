using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class SpringForce(
    RigidBody bodyA,
    RigidBody bodyB,
    Vector3 localAnchorA,
    Vector3 localAnchorB,
    float stiffness,
    float damping,
    float restLength) : ForceGenerator
{
    private const float MinLength = 1e-6f;
    public override RigidBody Body => bodyA;
    public override Vector3 LocalApplicationPoint => localAnchorA;
    
    public override Vector3 CalculateWorldForce() => CalculateWorldForceOnBodyA();

    private Vector3 CalculateWorldForceOnBodyA()
    {
        var displacement = CalculateAnchorsDisplacement();
        var length = displacement.Length();

        if (length < MinLength) return Vector3.Zero;

        var springDirection = displacement / length;
        var elasticStrength = CalculateElasticStrength(length);
        var dampingStrength = CalculateDampingStrength(springDirection);

        return (elasticStrength + dampingStrength) * springDirection;
    }

    private Vector3 CalculateAnchorsDisplacement()
    {
        var worldAnchorA = bodyA.Transform.TransformPoint(localAnchorA);
        var worldAnchorB = bodyB.Transform.TransformPoint(localAnchorB);
        return worldAnchorB - worldAnchorA;
    }

    private float CalculateElasticStrength(float length)
    {
        var extension = length - restLength;
        return stiffness * extension;
    }

    private float CalculateDampingStrength(Vector3 springDirection)
    {
        var velocityB = bodyB.GetVelocityAtPoint(localAnchorB);
        var velocityA = bodyA.GetVelocityAtPoint(localAnchorA);
        var relativeVelocity = velocityB - velocityA;

        var separationSpeed = Vector3.Dot(relativeVelocity, springDirection);
        return separationSpeed * damping;
    }
}