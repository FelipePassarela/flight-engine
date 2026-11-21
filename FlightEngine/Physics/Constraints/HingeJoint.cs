using System.Numerics;
using FlightEngine.Numerics;

namespace FlightEngine.Physics.Constraints;

/// <summary>
/// Keeps two body-local anchors coincident and aligns the body-local axes in world space.
/// Allows relative rotation around the shared axis.
/// </summary>
public class HingeJoint : Constraint
{
    private const int FirstAngularRow = AnchorCoincidenceRows.Count;
    private const int SecondAngularRow = FirstAngularRow + 1;
    private const int Dimension = SecondAngularRow + 1;
    private const float ErrorReduction = 0.1f;
    private const float AlignmentSineThreshold = 1e-6f;

    private readonly BodyPairConstraintSystem _system = new(Dimension);
    private Vector3 _firstTangent;
    private Vector3 _secondTangent;
    private bool _prepared;

    public HingeJoint(RigidBody bodyA, RigidBody bodyB,
        Vector3 localAnchorA, Vector3 localAnchorB, Vector3 localAxisA, Vector3 localAxisB)
    {
        ValidateBodies(bodyA, bodyB);
        ValidateAnchor(localAnchorA, nameof(localAnchorA));
        ValidateAnchor(localAnchorB, nameof(localAnchorB));

        BodyA = bodyA;
        BodyB = bodyB;
        LocalAnchorA = localAnchorA;
        LocalAnchorB = localAnchorB;
        LocalAxisA = NormalizeAxis(localAxisA, nameof(localAxisA));
        LocalAxisB = NormalizeAxis(localAxisB, nameof(localAxisB));
    }

    public RigidBody BodyA { get; }
    public RigidBody BodyB { get; }
    public Vector3 LocalAnchorA { get; }
    public Vector3 LocalAnchorB { get; }
    public Vector3 LocalAxisA { get; }
    public Vector3 LocalAxisB { get; }

    public override void Prepare(float deltaTime)
    {
        _prepared = false;
        if (!deltaTime.IsPositiveFinite() || (BodyA.HasInfiniteMass && BodyB.HasInfiniteMass))
            return;

        var biasScale = ErrorReduction / deltaTime;
        AnchorCoincidenceRows.Set(_system, BodyA, LocalAnchorA, BodyB, LocalAnchorB, biasScale);
        SetAxisAlignmentRows(biasScale);
        _system.Prepare(BodyA, BodyB);
        _prepared = true;
    }

    public override void SolveVelocity(float deltaTime)
    {
        if (!_prepared || !deltaTime.IsPositiveFinite())
            return;

        Span<double> impulseComponents = stackalloc double[Dimension];
        _system.Solve(BodyA, BodyB, impulseComponents);
        AnchorCoincidenceRows.ApplyImpulse(BodyA, LocalAnchorA, BodyB, LocalAnchorB, impulseComponents);
        ApplyAlignmentImpulse(impulseComponents);
    }

    private void SetAxisAlignmentRows(float biasScale)
    {
        var worldAxisA = BodyA.Transform.TransformDirection(LocalAxisA);
        var worldAxisB = BodyB.Transform.TransformDirection(LocalAxisB);
        _firstTangent = CalculatePerpendicular(worldAxisA);
        _secondTangent = Vector3.Cross(worldAxisA, _firstTangent);

        var rotationError = CalculateAlignmentError(worldAxisA, worldAxisB, _firstTangent);
        var angularBias = rotationError * biasScale;
        _system.SetRelativeAngularRow(FirstAngularRow, _firstTangent, angularBias);
        _system.SetRelativeAngularRow(SecondAngularRow, _secondTangent, angularBias);
    }

    private static Vector3 CalculatePerpendicular(Vector3 axis)
    {
        // Choose a reference that stays away from parallel to the axis.
        var reference = MathF.Abs(axis.X) < 0.8f ? Vector3.UnitX : Vector3.UnitY;
        return Vector3.Normalize(Vector3.Cross(axis, reference));
    }

    private static Vector3 CalculateAlignmentError(Vector3 axisA, Vector3 axisB, Vector3 fallbackTangent)
    {
        var cross = Vector3.Cross(axisA, axisB);
        var sine = cross.Length();
        var cosine = Vector3.Dot(axisA, axisB);
        if (sine < AlignmentSineThreshold)
        {
            // At 180 degrees, any perpendicular axis defines a shortest rotation.
            // Near zero degrees, the cross product approximates the rotation vector.
            return cosine < 0f ? MathF.PI * fallbackTangent : cross;
        }

        var angle = MathF.Atan2(sine, cosine);
        return cross * (angle / sine);
    }

    private void ApplyAlignmentImpulse(ReadOnlySpan<double> impulseComponents)
    {
        var angularImpulse = (float)impulseComponents[FirstAngularRow] * _firstTangent +
                             (float)impulseComponents[SecondAngularRow] * _secondTangent;
        BodyA.ApplyAngularImpulse(-angularImpulse);
        BodyB.ApplyAngularImpulse(angularImpulse);
    }

    private static void ValidateBodies(RigidBody bodyA, RigidBody bodyB)
    {
        ArgumentNullException.ThrowIfNull(bodyA);
        ArgumentNullException.ThrowIfNull(bodyB);
        if (ReferenceEquals(bodyA, bodyB))
            throw new ArgumentException("A hinge joint requires two different bodies.", nameof(bodyB));
    }

    private static void ValidateAnchor(Vector3 anchor, string parameterName)
    {
        if (!anchor.AllComponentsAreFinite())
            throw new ArgumentOutOfRangeException(parameterName, anchor, "Anchor coordinates must be finite.");
    }

    private static Vector3 NormalizeAxis(Vector3 axis, string parameterName)
    {
        if (!axis.AllComponentsAreFinite() || axis == Vector3.Zero)
            throw new ArgumentOutOfRangeException(parameterName, axis, "The hinge axis must be finite and nonzero.");

        // Rescale before measuring length so very small and very large axes can be normalized.
        var absoluteAxis = Vector3.Abs(axis);
        var largestComponent = MathF.Max(absoluteAxis.X, MathF.Max(absoluteAxis.Y, absoluteAxis.Z));
        return Vector3.Normalize(axis / largestComponent);
    }
}
