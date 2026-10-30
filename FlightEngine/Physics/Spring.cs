using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class Spring(
    RigidBody bodyA,
    RigidBody bodyB,
    Vector3 localAnchorA = default,
    Vector3 localAnchorB = default,
    float stiffness = 10.0f,
    float damping = 1.0f,
    float restLength = 1.0f) : Component
{
    public float Stiffness { get; set; } = stiffness;
    public float Damping { get; set; } = damping;
    public float RestLength { get; set; } = restLength;
    public Vector3 LocalAnchorA { get; set; } = localAnchorA;
    public Vector3 LocalAnchorB { get; set; } = localAnchorB;
    private ForceApplier _forceApplierA = null!;
    private ForceApplier _forceApplierB = null!;

    public Vector3 WorldAnchorA => bodyA.Transform.TransformPoint(LocalAnchorA);
    public Vector3 WorldAnchorB => bodyB.Transform.TransformPoint(LocalAnchorB);

    public override void OnInitialize()
    {
        var gameObjectA = bodyA.GameObject;
        var gameObjectB = bodyB.GameObject;
        _forceApplierA = gameObjectA.GetComponent<ForceApplier>() ??
                         throw new InvalidOperationException("GameObject 'A' requires a ForceApplier component.");
        _forceApplierB = gameObjectB.GetComponent<ForceApplier>() ??
                         throw new InvalidOperationException("GameObject 'B' requires a ForceApplier component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        var springForceOnA = new SpringForce(
            bodyA,
            bodyB,
            LocalAnchorA,
            LocalAnchorB,
            Stiffness,
            Damping,
            RestLength);
        var springForceOnB = new SpringForce(
            bodyB,
            bodyA,
            LocalAnchorB,
            LocalAnchorA,
            Stiffness,
            Damping,
            RestLength);
        _forceApplierA.AddForce(springForceOnA);
        _forceApplierB.AddForce(springForceOnB);
    }
}