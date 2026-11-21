using System.Numerics;
using FlightEngine.Numerics;

namespace FlightEngine.Physics.Constraints;

public class DistanceJoint(
    RigidBody bodyA,
    RigidBody bodyB,
    Vector3 localAnchorA,
    Vector3 localAnchorB,
    float targetDistance,
    float errorReduction = 0.1f) : Constraint
{
    public RigidBody BodyA { get; } = bodyA;
    public RigidBody BodyB { get; } = bodyB;
    public Vector3 LocalAnchorA { get; } = localAnchorA;
    public Vector3 LocalAnchorB { get; } = localAnchorB;

    public float TargetDistance { get; } = targetDistance.IsPositiveFinite()
        ? targetDistance
        : throw new ArgumentOutOfRangeException(
            nameof(targetDistance), targetDistance, "Target distance must be positive and finite.");

    private readonly record struct PreparedStep(Vector3 Direction, float PositionBias, float EffectiveMass);

    private PreparedStep? _preparedStep;

    public override void Prepare(float deltaTime)
    {
        _preparedStep = null;
        if (!deltaTime.IsPositiveFinite())
            return;

        var anchorA = BodyA.Transform.TransformPoint(LocalAnchorA);
        var anchorB = BodyB.Transform.TransformPoint(LocalAnchorB);
        var separation = anchorB - anchorA;
        var currentDistance = separation.Length();
        // Fallback for coincident anchors
        var direction = currentDistance > 0.0f ? separation / currentDistance : Vector3.UnitX;

        var effectiveMass = GetEffectiveMass(direction);
        if (!effectiveMass.IsPositiveFinite())
            return;

        var distanceError = currentDistance - TargetDistance;
        var positionBias = errorReduction * distanceError / deltaTime;
        _preparedStep = new PreparedStep(direction, positionBias, effectiveMass);
    }

    private float GetEffectiveMass(Vector3 direction)
    {
        var angularContributionA = GetAngularInverseMassContribution(BodyA, LocalAnchorA, direction);
        var angularContributionB = GetAngularInverseMassContribution(BodyB, LocalAnchorB, direction);
        var inverseEffectiveMass = BodyA.InverseMass + BodyB.InverseMass + angularContributionA + angularContributionB;
        return inverseEffectiveMass.IsPositiveFinite() ? 1.0f / inverseEffectiveMass : 0.0f;
    }

    private static float GetAngularInverseMassContribution(RigidBody body, Vector3 localAnchor, Vector3 direction)
    {
        var offset = body.Transform.TransformVector(localAnchor - body.CenterOfMass);
        var angularAxis = Vector3.Cross(offset, direction);
        return body.InverseMomentOfInertia * angularAxis.LengthSquared();
    }

    public override void SolveVelocity(float deltaTime)
    {
        if (_preparedStep is not { } step || !deltaTime.IsPositiveFinite())
            return;

        var relativeSpeed = GetRelativeVelocityAlong(step.Direction);
        var impulse = GetImpulse(step, relativeSpeed);
        BodyA.ApplyImpulse(-impulse, LocalAnchorA);
        BodyB.ApplyImpulse(impulse, LocalAnchorB);
    }

    private float GetRelativeVelocityAlong(Vector3 direction)
    {
        var velocityA = BodyA.GetVelocityAtPoint(LocalAnchorA);
        var velocityB = BodyB.GetVelocityAtPoint(LocalAnchorB);
        return Vector3.Dot(velocityB - velocityA, direction);
    }

    private static Vector3 GetImpulse(PreparedStep step, float relativeSpeed)
    {
        var scalarImpulse = -step.EffectiveMass * (relativeSpeed + step.PositionBias);
        return scalarImpulse * step.Direction;
    }
}
