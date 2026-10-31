using System.Numerics;

namespace FlightEngine.Physics.Constraints;

public class FixedJoint(RigidBody bodyA, RigidBody bodyB, Vector3 contactPoint) : Constraint
{
    private const int Dimension = 6;
    private const float ErrorReduction = 0.1f;
    private const float SmallAngleThreshold = 1e-6f;
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

        _relativeOrientation = Quaternion.Normalize(
            Quaternion.Inverse(bodyA.Transform.Orientation) * bodyB.Transform.Orientation);
    }

    public override void Prepare(float deltaTime)
    {
        _prepared = false;
        if (!float.IsFinite(deltaTime) || deltaTime <= 0.0f ||
            (bodyA.HasInfiniteMass && bodyB.HasInfiniteMass)) return;

        var transformA = bodyA.Transform;
        var transformB = bodyB.Transform;

        var rA = transformA.TransformDirection(_localAnchorA - bodyA.CenterOfMass);
        var rB = transformB.TransformDirection(_localAnchorB - bodyB.CenterOfMass);
        var anchorA = transformA.TransformPoint(_localAnchorA);
        var anchorB = transformB.TransformPoint(_localAnchorB);

        var biasScale = ErrorReduction / deltaTime;
        var positionBias = (anchorB - anchorA) * biasScale;
        var angularBias = GetRotationError() * biasScale;

        BuildConstraintRows(rA, rB, positionBias, angularBias);
        _system.Prepare(bodyA, bodyB);
        _prepared = true;
    }

    public override void SolveVelocity(float deltaTime)
    {
        if (!_prepared || !float.IsFinite(deltaTime) || deltaTime <= 0.0f)
            return;

        Span<double> impulse = stackalloc double[Dimension];
        _system.Solve(bodyA, bodyB, impulse);
        AddImpulses(impulse);
    }

    private Vector3 GetRotationError()
    {
        var desiredOrientation = bodyA.Transform.Orientation * _relativeOrientation;
        var error = Quaternion.Normalize(bodyB.Transform.Orientation * Quaternion.Inverse(desiredOrientation));
        // q and -q represent the same rotation; select the shortest correction.
        if (error.W < 0.0f) error = Quaternion.Negate(error);

        var axis = new Vector3(error.X, error.Y, error.Z);
        var length = axis.Length();
        return length < SmallAngleThreshold
            ? 2.0f * axis
            : axis * (2.0f * MathF.Atan2(length, error.W) / length);
    }

    private void BuildConstraintRows(Vector3 rA, Vector3 rB, Vector3 positionBias, Vector3 angularBias)
    {
        for (var i = 0; i < 3; i++)
        {
            var unit = i switch { 0 => Vector3.UnitX, 1 => Vector3.UnitY, _ => Vector3.UnitZ };
            _system.SetRow(
                i,
                unit,
                Vector3.Cross(unit, rA),
                -Vector3.Cross(unit, rB),
                Vector3.Dot(unit, positionBias));
            _system.SetRow(i + 3, Vector3.Zero, -unit, unit, Vector3.Dot(unit, angularBias));
        }
    }

    private void AddImpulses(ReadOnlySpan<double> impulse)
    {
        var linearImpulse = new Vector3((float)impulse[0], (float)impulse[1], (float)impulse[2]);
        var angularImpulse = new Vector3((float)impulse[3], (float)impulse[4], (float)impulse[5]);
        bodyA.AddImpulse(-linearImpulse, _localAnchorA);
        bodyA.AddAngularImpulse(-angularImpulse);
        bodyB.AddImpulse(linearImpulse, _localAnchorB);
        bodyB.AddAngularImpulse(angularImpulse);
    }
}