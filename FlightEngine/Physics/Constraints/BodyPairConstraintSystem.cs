using System.Numerics;

namespace FlightEngine.Physics.Constraints;

// Solves coupled constraint rows for two bodies. The linear Jacobian acts on vB - vA.
internal sealed class BodyPairConstraintSystem(int dimension)
{
    private readonly ConstraintRow[] _rows = new ConstraintRow[dimension];
    private readonly double[,] _massFactor = new double[dimension, dimension];

    public int Dimension => dimension;

    public void SetRow(int index, Vector3 linear, Vector3 angularA, Vector3 angularB, double bias)
    {
        _rows[index] = new ConstraintRow(linear, angularA, angularB, bias);
    }

    public void SetRelativeAngularRow(int index, Vector3 axis, Vector3 angularBias)
    {
        var bias = Vector3.Dot(axis, angularBias);
        SetRow(index, Vector3.Zero, -axis, axis, bias);
    }

    public void Prepare(RigidBody bodyA, RigidBody bodyB)
    {
        // Cholesky factorization of K = J M^-1 J^T, cached for all solver iterations.
        Array.Clear(_massFactor);
        for (var row = 0; row < dimension; row++)
        {
            for (var column = 0; column <= row; column++)
            {
                var value = GetEffectiveMass(_rows[row], _rows[column], bodyA, bodyB);
                for (var k = 0; k < column; k++)
                    value -= _massFactor[row, k] * _massFactor[column, k];

                if (row == column)
                {
                    _massFactor[row, column] = Math.Sqrt(Math.Max(0.0, value));
                    continue;
                }

                // Zero angular inverse inertia can leave an unresponsive row.
                var diagonal = _massFactor[column, column];
                _massFactor[row, column] = diagonal > 0.0 ? value / diagonal : 0.0;
            }
        }
    }

    public void Solve(RigidBody bodyA, RigidBody bodyB, Span<double> impulse)
    {
        BuildRightHandSide(bodyA, bodyB, impulse);
        ForwardSubstitute(impulse, dimension);
        BackSubstitute(impulse, dimension);
    }

    // The last row represents a bounded velocity and has no bias of its own.
    public void SolveWithBoundedLastRow(
        RigidBody bodyA,
        RigidBody bodyB,
        Span<double> impulse,
        (double Minimum, double Maximum) velocityBounds,
        ref double accumulatedImpulse)
    {
        BuildRightHandSide(bodyA, bodyB, impulse);
        var boundedRow = dimension - 1;
        ForwardSubstitute(impulse, boundedRow);
        var reducedRightHandSide = SubtractPreviousRows(impulse, boundedRow);
        impulse[boundedRow] = SolveBoundedImpulse(
            boundedRow, -reducedRightHandSide, velocityBounds, ref accumulatedImpulse);
        BackSubstitute(impulse, boundedRow);
    }

    private void BuildRightHandSide(RigidBody bodyA, RigidBody bodyB, Span<double> impulse)
    {
        // K * lambda = -(J * V + bias), using the latest velocities each iteration.
        var relativeVelocity = bodyB.Velocity - bodyA.Velocity;
        for (var i = 0; i < dimension; i++)
        {
            var row = _rows[i];
            impulse[i] = -(Dot(row.Linear, relativeVelocity)
                           + Dot(row.AngularA, bodyA.AngularVelocity)
                           + Dot(row.AngularB, bodyB.AngularVelocity) + row.Bias);
        }
    }

    private void ForwardSubstitute(Span<double> impulse, int rowCount)
    {
        for (var i = 0; i < rowCount; i++)
        {
            var reducedRightHandSide = SubtractPreviousRows(impulse, i);
            var diagonal = _massFactor[i, i];
            impulse[i] = diagonal > 0.0 ? reducedRightHandSide / diagonal : 0.0;
        }
    }

    private double SubtractPreviousRows(ReadOnlySpan<double> impulse, int row)
    {
        var residual = impulse[row];
        for (var column = 0; column < row; column++)
            residual -= _massFactor[row, column] * impulse[column];
        return residual;
    }

    private double SolveBoundedImpulse(
        int row,
        double velocityAfterBilateralSolve,
        (double Minimum, double Maximum) velocityBounds,
        ref double accumulatedImpulse)
    {
        // The square of the final Cholesky pivot is the inverse mass after accounting for the bilateral rows.
        var diagonal = _massFactor[row, row];
        var inverseEffectiveMass = diagonal * diagonal;
        if (!(inverseEffectiveMass > 0.0) || !double.IsFinite(inverseEffectiveMass))
            return 0.0;

        // Recover the velocity without this step's stop impulse so later iterations can release it.
        var freeVelocity = velocityAfterBilateralSolve - inverseEffectiveMass * accumulatedImpulse;
        var allowedVelocity = Math.Clamp(freeVelocity, velocityBounds.Minimum, velocityBounds.Maximum);
        var nextImpulse = (allowedVelocity - freeVelocity) / inverseEffectiveMass;
        var impulseChange = nextImpulse - accumulatedImpulse;
        accumulatedImpulse = nextImpulse;
        return impulseChange;
    }

    private void BackSubstitute(Span<double> impulse, int rowCount)
    {
        for (var i = rowCount - 1; i >= 0; i--)
        {
            for (var j = i + 1; j < dimension; j++)
                impulse[i] -= _massFactor[j, i] * impulse[j];
            var diagonal = _massFactor[i, i];
            impulse[i] = diagonal > 0.0 ? impulse[i] / diagonal : 0.0;
        }
    }

    private static double GetEffectiveMass(ConstraintRow row, ConstraintRow column,
        RigidBody bodyA, RigidBody bodyB) =>
        ((double)bodyA.InverseMass + bodyB.InverseMass) * Dot(row.Linear, column.Linear)
        + bodyA.InverseMomentOfInertia * Dot(row.AngularA, column.AngularA)
        + bodyB.InverseMomentOfInertia * Dot(row.AngularB, column.AngularB);

    // Preserve double precision when off-center anchors couple the rows.
    private static double Dot(Vector3 a, Vector3 b) => (double)a.X * b.X + (double)a.Y * b.Y + (double)a.Z * b.Z;

    private readonly record struct ConstraintRow(Vector3 Linear, Vector3 AngularA, Vector3 AngularB, double Bias);
}