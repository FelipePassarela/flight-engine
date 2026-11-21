using System.Numerics;
using FlightEngine.Numerics;

namespace FlightEngine.Physics.Constraints;

public class BallJoint : Constraint
{
    private const float ErrorReduction = 0.1f;
    private readonly BodyPairConstraintSystem _system = new(AnchorCoincidenceRows.Count);
    private bool _prepared;

    public BallJoint(RigidBody bodyA, RigidBody bodyB, Vector3 localAnchorA, Vector3 localAnchorB)
    {
        ValidateBodies(bodyA, bodyB);
        ValidateAnchor(localAnchorA, nameof(localAnchorA));
        ValidateAnchor(localAnchorB, nameof(localAnchorB));

        BodyA = bodyA;
        BodyB = bodyB;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
    }

    public RigidBody BodyA { get; }
    public RigidBody BodyB { get; }
    public Vector3 LocalAnchorA { get; }
    public Vector3 LocalAnchorB { get; }

    public override void Prepare(float deltaTime)
    {
        _prepared = false;
        if (!deltaTime.IsPositiveFinite() || (BodyA.HasInfiniteMass && BodyB.HasInfiniteMass))
            return;

        var biasScale = ErrorReduction / deltaTime;
        AnchorCoincidenceRows.Set(_system, BodyA, LocalAnchorA, BodyB, LocalAnchorB, biasScale);
        _system.Prepare(BodyA, BodyB);
        _prepared = true;
    }

    public override void SolveVelocity(float deltaTime)
    {
        if (!_prepared || !deltaTime.IsPositiveFinite())
            return;

        Span<double> impulseComponents = stackalloc double[AnchorCoincidenceRows.Count];
        _system.Solve(BodyA, BodyB, impulseComponents);
        ApplyAnchorImpulse(impulseComponents);
    }

    private void ApplyAnchorImpulse(ReadOnlySpan<double> impulseComponents)
    {
        var impulse = new Vector3((float)impulseComponents[0], (float)impulseComponents[1], (float)impulseComponents[2]);
        BodyA.ApplyImpulse(-impulse, LocalAnchorA);
        BodyB.ApplyImpulse(impulse, LocalAnchorB);
    }

    private static void ValidateBodies(RigidBody bodyA, RigidBody bodyB)
    {
        ArgumentNullException.ThrowIfNull(bodyA);
        ArgumentNullException.ThrowIfNull(bodyB);
        if (ReferenceEquals(bodyA, bodyB))
            throw new ArgumentException("A ball joint requires two different bodies.", nameof(bodyB));
    }

    private static void ValidateAnchor(Vector3 localAnchor, string parameterName)
    {
        if (!localAnchor.HasFiniteComponents())
            throw new ArgumentOutOfRangeException(parameterName, localAnchor, "Anchor coordinates must be finite.");
    }
}
