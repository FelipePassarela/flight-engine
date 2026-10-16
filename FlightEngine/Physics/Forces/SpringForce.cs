using System.Numerics;

namespace FlightEngine.Physics.Forces;

public class SpringForce(
    float stiffness,
    float damping,
    float restLength,
    Vector3 thisLocalAnchor,
    Vector3 otherWorldAnchor) : ForceGenerator
{
    private const float MinLength = 1e-6f;
    public override Vector3 LocalApplicationPoint => thisLocalAnchor;

    public override Vector3 CalculateWorldForce(RigidBody body)
    {
        var thisWorldAnchor = body.Transform.TransformPoint(thisLocalAnchor);
        var displacement = otherWorldAnchor - thisWorldAnchor;
        var length = displacement.Length();

        if (length < MinLength) return Vector3.Zero;

        var springDirection = displacement / length;
        var elasticForce = CalculateElasticStrength(length);
        var dampingForce = CalculateDampingStrength(body, springDirection);

        return (elasticForce + dampingForce) * springDirection;
    }

    private float CalculateElasticStrength(float length)
    {
        var extension = length - restLength;
        return stiffness * extension;
    }

    private float CalculateDampingStrength(RigidBody body, Vector3 springDirection)
    {
        var anchorVelocity = body.GetVelocityAtPoint(thisLocalAnchor);
        var speedAlongSpring = Vector3.Dot(anchorVelocity, springDirection);
        return -speedAlongSpring * damping;
    }
}