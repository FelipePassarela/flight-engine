using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class SpringForce(
    RigidBody thisBody,
    RigidBody otherBody,
    Vector3 thisLocalAnchor,
    Vector3 otherLocalAnchor,
    float stiffness,
    float damping,
    float restLength) : PairForceGenerator
{
    public override RigidBody ThisBody => thisBody;
    public override RigidBody OtherBody => otherBody;
    public override Vector3 LocalApplicationPoint => thisLocalAnchor;
    public override Vector3 OtherLocalApplicationPoint => otherLocalAnchor;
    private const float MinLength = 1e-6f;

    public override Vector3 CalculateWorldForceOnThis()
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
        var thisWorldAnchor = thisBody.Transform.TransformPoint(thisLocalAnchor);
        var otherWorldAnchor = otherBody.Transform.TransformPoint(otherLocalAnchor);
        return otherWorldAnchor - thisWorldAnchor;
    }

    private float CalculateElasticStrength(float length)
    {
        var extension = length - restLength;
        return stiffness * extension;
    }

    private float CalculateDampingStrength(Vector3 springDirection)
    {
        var otherVelocity = otherBody.GetVelocityAtPoint(otherLocalAnchor);
        var thisVelocity = thisBody.GetVelocityAtPoint(thisLocalAnchor);
        var relativeVelocity = otherVelocity - thisVelocity;

        var separationSpeed = Vector3.Dot(relativeVelocity, springDirection);
        return separationSpeed * damping;
    }
}