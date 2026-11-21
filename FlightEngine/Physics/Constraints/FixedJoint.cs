using System.Numerics;
using FlightEngine.Numerics;

namespace FlightEngine.Physics.Constraints;

public class FixedJoint(RigidBody bodyA, RigidBody bodyB, Vector3 contactPoint) : Constraint
{
    private const int FirstOrientationRow = AnchorCoincidenceRows.Count;
    private const int OrientationRowCount = 3;
    private const int Dimension = FirstOrientationRow + OrientationRowCount;
    private const float ErrorReduction = 0.1f;
    private const float HalfAngleSineThreshold = 1e-6f;
    private Vector3 _localAnchorA;
    private Vector3 _localAnchorB;
    private Quaternion _relativeOrientation;
    private bool _prepared;
    private readonly BodyPairConstraintSystem _system = new(Dimension);

    public Vector3 ContactPoint => bodyA.Transform.TransformPoint(_localAnchorA);
    public RigidBody BodyA => bodyA;
    public RigidBody BodyB => bodyB;

    public override void OnInitialize()
    {
        if (ReferenceEquals(bodyA, bodyB))
            throw new ArgumentException("A fixed joint requires two different bodies.");

        _localAnchorA = bodyA.Transform.InverseTransformPoint(contactPoint);
        _localAnchorB = bodyB.Transform.InverseTransformPoint(contactPoint);
        _relativeOrientation = CalculateRelativeOrientation();
    }

    public override void Prepare(float deltaTime)
    {
        _prepared = false;
        if (!deltaTime.IsPositiveFinite() || (bodyA.HasInfiniteMass && bodyB.HasInfiniteMass))
            return;

        var biasScale = ErrorReduction / deltaTime;
        AnchorCoincidenceRows.Set(_system, bodyA, _localAnchorA, bodyB, _localAnchorB, biasScale);
        SetOrientationRows(biasScale);
        _system.Prepare(bodyA, bodyB);
        _prepared = true;
    }

    public override void SolveVelocity(float deltaTime)
    {
        if (!_prepared || !deltaTime.IsPositiveFinite())
            return;

        Span<double> impulseComponents = stackalloc double[Dimension];
        _system.Solve(bodyA, bodyB, impulseComponents);
        AnchorCoincidenceRows.ApplyImpulse(bodyA, _localAnchorA, bodyB, _localAnchorB, impulseComponents);
        ApplyOrientationImpulse(impulseComponents);
    }

    private void ApplyOrientationImpulse(ReadOnlySpan<double> impulseComponents)
    {
        var angularImpulse = new Vector3(
            (float)impulseComponents[FirstOrientationRow],
            (float)impulseComponents[FirstOrientationRow + 1],
            (float)impulseComponents[FirstOrientationRow + 2]);
        bodyA.ApplyAngularImpulse(-angularImpulse);
        bodyB.ApplyAngularImpulse(angularImpulse);
    }

    private Quaternion CalculateRelativeOrientation()
    {
        var inverseOrientationA = Quaternion.Inverse(bodyA.Transform.Orientation);
        var orientationB = bodyB.Transform.Orientation;
        return Quaternion.Normalize(inverseOrientationA * orientationB);
    }

    private Vector3 GetRotationError()
    {
        var desiredOrientation = bodyA.Transform.Orientation * _relativeOrientation;
        var currentOrientation = bodyB.Transform.Orientation;
        var inverseDesiredOrientation = Quaternion.Inverse(desiredOrientation);
        var errorRotation = Quaternion.Normalize(currentOrientation * inverseDesiredOrientation);
        return ToShortestRotationVector(errorRotation);
    }

    private static Vector3 ToShortestRotationVector(Quaternion unitRotation)
    {
        // q and -q represent the same rotation. Nonnegative W selects an angle in [0, pi].
        var shortestRotation = unitRotation.W < 0.0f ? Quaternion.Negate(unitRotation) : unitRotation;
        var vectorPart = new Vector3(shortestRotation.X, shortestRotation.Y, shortestRotation.Z);
        var sinHalfAngle = vectorPart.Length();

        // Near zero, sin(angle / 2) approaches angle / 2.
        if (sinHalfAngle < HalfAngleSineThreshold)
            return 2.0f * vectorPart;

        var angleRadians = 2.0f * MathF.Atan2(sinHalfAngle, shortestRotation.W);
        var scale = angleRadians / sinHalfAngle;
        return vectorPart * scale;
    }

    private void SetOrientationRows(float biasScale)
    {
        var angularBias = GetRotationError() * biasScale;
        for (var i = 0; i < OrientationRowCount; i++)
        {
            var axis = i switch { 0 => Vector3.UnitX, 1 => Vector3.UnitY, _ => Vector3.UnitZ };
            var rowIndex = FirstOrientationRow + i;
            _system.SetRelativeAngularRow(rowIndex, axis, angularBias);
        }
    }
}
