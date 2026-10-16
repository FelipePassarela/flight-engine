using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class Spring(
    Transform otherTransform,
    float stiffness = 10.0f,
    float damping = 1.0f,
    float restLength = 1.0f,
    Vector3 thisAnchorOffset = default,
    Vector3 otherAnchorOffset = default) : Component
{
    public float Stiffness { get; set; } = stiffness;
    public float Damping { get; set; } = damping;
    public float RestLength { get; set; } = restLength;
    public Vector3 ThisAnchorOffset { get; set; } = thisAnchorOffset;
    public Vector3 OtherAnchorOffset { get; set; } = otherAnchorOffset;
    private ForceApplier _forceApplier = null!;

    public Vector3 ThisWorldAnchor => GameObject.Transform.TransformPoint(ThisAnchorOffset);

    public Vector3 OtherWorldAnchor => otherTransform.TransformPoint(OtherAnchorOffset);

    public override void OnInitialize()
    {
        _forceApplier = GameObject.GetComponent<ForceApplier>() ??
                        throw new InvalidOperationException("Spring requires a ForceApplier component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        var springForce = new SpringForce(Stiffness, Damping, RestLength, ThisAnchorOffset, OtherWorldAnchor);
        _forceApplier.AddForce(springForce);
    }
}