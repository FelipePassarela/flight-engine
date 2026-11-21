using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics.Constraints;

internal sealed class HingeLimit(
    RigidBody bodyA, RigidBody bodyB, BodyPairConstraintSystem system, HingeAngleLimits limits)
{
    private const float RecoveryFraction = 0.1f;
    private const float AngleTolerance = 1e-6f;

    // The coupled solver constrains the axial velocity in its last row.
    private readonly int _rowIndex = system.Dimension - 1;
    private AngleReference? _angleReference;
    private Vector3 _axis;
    private (double Minimum, double Maximum) _velocityBounds;
    private double _accumulatedImpulse;

    public HingeAngleLimits Limits => limits;

    public void Prepare(Vector3 axis, Vector3 referenceTangent, Vector3 alignmentError, float deltaTime)
    {
        var angle = MeasureAngleFromNeutralPose(axis, referenceTangent, alignmentError);
        _axis = axis;
        _velocityBounds = CalculateVelocityBounds(angle, deltaTime);
        _accumulatedImpulse = 0.0;
        system.SetRelativeAngularRow(_rowIndex, axis, Vector3.Zero);
    }

    public void Solve(Span<double> impulseComponents) =>
        system.SolveWithBoundedLastRow(bodyA, bodyB, impulseComponents, _velocityBounds, ref _accumulatedImpulse);

    public Vector3 CalculateAngularImpulse(ReadOnlySpan<double> impulseComponents) =>
        (float)impulseComponents[_rowIndex] * _axis;

    private float MeasureAngleFromNeutralPose(Vector3 axis, Vector3 referenceTangent, Vector3 alignmentError)
    {
        var swingFromAToB = ToRotation(alignmentError);
        var transformA = bodyA.Transform;
        var transformB = bodyB.Transform;
        var reference = _angleReference ??= AngleReference.Capture(
            transformA, transformB, referenceTangent, swingFromAToB);
        return reference.Measure(transformA, transformB, axis, swingFromAToB);
    }

    private (double Minimum, double Maximum) CalculateVelocityBounds(float angle, float deltaTime)
    {
        var angleInRange = Math.Clamp(angle, limits.MinimumRadians, limits.MaximumRadians);
        var angleCorrection = (double)angleInRange - angle;
        if (Math.Abs(angleCorrection) > AngleTolerance)
        {
            // Decelerate during recovery so the correction cannot carry the joint across the interval.
            var recoverySpeed = angleCorrection * RecoveryFraction / deltaTime;
            return (recoverySpeed, recoverySpeed);
        }

        var minimumSpeed = ((double)limits.MinimumRadians - angleInRange) / deltaTime;
        var maximumSpeed = ((double)limits.MaximumRadians - angleInRange) / deltaTime;
        return (minimumSpeed, maximumSpeed);
    }

    private static Quaternion ToRotation(Vector3 rotationVector)
    {
        var angle = rotationVector.Length();
        return angle > 0f ? Quaternion.CreateFromAxisAngle(rotationVector / angle, angle) : Quaternion.Identity;
    }

    private readonly record struct AngleReference(Vector3 LocalReferenceA, Vector3 LocalReferenceB)
    {
        public static AngleReference Capture(Transform transformA, Transform transformB, Vector3 worldReferenceA,
            Quaternion swingFromAToB)
        {
            var worldReferenceB = Vector3.Transform(worldReferenceA, swingFromAToB);
            return new AngleReference(
                transformA.InverseTransformDirection(worldReferenceA),
                transformB.InverseTransformDirection(worldReferenceB));
        }

        public float Measure(Transform transformA, Transform transformB, Vector3 worldAxisA, Quaternion swingFromAToB)
        {
            var worldReferenceA = transformA.TransformDirection(LocalReferenceA);
            var worldReferenceB = transformB.TransformDirection(LocalReferenceB);
            // Undo the swing between the axes before measuring the signed twist in A's plane.
            var alignedReferenceB = Vector3.Transform(worldReferenceB, Quaternion.Conjugate(swingFromAToB));
            var sine = Vector3.Dot(worldAxisA, Vector3.Cross(worldReferenceA, alignedReferenceB));
            var cosine = Vector3.Dot(worldReferenceA, alignedReferenceB);
            return MathF.Atan2(sine, cosine);
        }
    }
}
