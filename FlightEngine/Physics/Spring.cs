using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class Spring(
    RigidBody otherBody,
    float stiffness = 10.0f,
    float damping = 1.0f,
    float restLength = 1.0f,
    Vector3 thisLocalAnchor = default,
    Vector3 otherLocalAnchor = default) : Component
{
    public float Stiffness { get; set; } = stiffness;
    public float Damping { get; set; } = damping;
    public float RestLength { get; set; } = restLength;
    public Vector3 ThisLocalAnchor { get; set; } = thisLocalAnchor;
    public Vector3 OtherLocalAnchor { get; set; } = otherLocalAnchor;
    private ForceApplier _forceApplier = null!;
    private RigidBody _thisBody = null!;

    public Vector3 ThisWorldAnchor => GameObject.Transform.TransformPoint(ThisLocalAnchor);

    public Vector3 OtherWorldAnchor => otherBody.Transform.TransformPoint(OtherLocalAnchor);

    public override void OnInitialize()
    {
        _forceApplier = GameObject.GetComponent<ForceApplier>() ??
                        throw new InvalidOperationException("Spring requires a ForceApplier component.");
        _thisBody = GameObject.GetComponent<RigidBody>() ??
                    throw new InvalidOperationException("Spring requires a RigidBody component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        var springForce = new SpringForce(
            _thisBody,
            otherBody,
            ThisLocalAnchor,
            OtherLocalAnchor,
            Stiffness,
            Damping,
            RestLength);
        _forceApplier.AddForce(springForce);
    }
}