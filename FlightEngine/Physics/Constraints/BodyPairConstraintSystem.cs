using System.Numerics;

namespace FlightEngine.Physics.Constraints;

// Solves coupled constraint rows for two bodies. The linear Jacobian acts on vB - vA.
internal sealed class BodyPairConstraintSystem(int dimension)
{
    private readonly ConstraintRow[] _rows = new ConstraintRow[dimension];
    private readonly double[,] _massFactor = new double[dimension, dimension];

    public void SetRow(int index, Vector3 linear, Vector3 angularA, Vector3 angularB, double bias)
    {
        _rows[index] = new ConstraintRow(linear, angularA, angularB, bias);
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
        // K * lambda = -(J * V + bias), using the latest velocities each iteration.
        var relativeVelocity = bodyB.Velocity - bodyA.Velocity;
        for (var i = 0; i < dimension; i++)
        {
            var row = _rows[i];
            impulse[i] = -(Dot(row.Linear, relativeVelocity)
                           + Dot(row.AngularA, bodyA.AngularVelocity)
                           + Dot(row.AngularB, bodyB.AngularVelocity) + row.Bias);
        }

        SolveEffectiveMass(impulse);
    }

    private void SolveEffectiveMass(Span<double> impulse)
    {
        // Forward/back substitution through the cached Cholesky factor.
        for (var i = 0; i < dimension; i++)
        {
            for (var j = 0; j < i; j++)
                impulse[i] -= _massFactor[i, j] * impulse[j];
            var diagonal = _massFactor[i, i];
            impulse[i] = diagonal > 0.0 ? impulse[i] / diagonal : 0.0;
        }

        for (var i = dimension - 1; i >= 0; i--)
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