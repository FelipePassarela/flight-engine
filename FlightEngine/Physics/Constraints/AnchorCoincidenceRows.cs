using System.Numerics;

namespace FlightEngine.Physics.Constraints;

internal static class AnchorCoincidenceRows
{
    public const int Count = 3;

    public static void Set(
        BodyPairConstraintSystem system,
        RigidBody bodyA,
        Vector3 localAnchorA,
        RigidBody bodyB,
        Vector3 localAnchorB,
        float biasScale)
    {
        var transformA = bodyA.Transform;
        var transformB = bodyB.Transform;
        var offsetA = transformA.TransformVector(localAnchorA - bodyA.CenterOfMass);
        var offsetB = transformB.TransformVector(localAnchorB - bodyB.CenterOfMass);
        var worldAnchorA = transformA.TransformPoint(localAnchorA);
        var worldAnchorB = transformB.TransformPoint(localAnchorB);

        var positionError = worldAnchorB - worldAnchorA;
        var positionBias = positionError * biasScale;
        SetRowsForAxes(system, offsetA, offsetB, positionBias);
    }

    public static void ApplyImpulse(
        RigidBody bodyA,
        Vector3 localAnchorA,
        RigidBody bodyB,
        Vector3 localAnchorB,
        ReadOnlySpan<double> impulseComponents)
    {
        // The X, Y and Z rows form the world-space impulse on body B.
        var impulse = new Vector3(
            (float)impulseComponents[0], (float)impulseComponents[1], (float)impulseComponents[2]);
        bodyA.ApplyImpulse(-impulse, localAnchorA);
        bodyB.ApplyImpulse(impulse, localAnchorB);
    }

    private static void SetRowsForAxes(
        BodyPairConstraintSystem system, Vector3 offsetA, Vector3 offsetB, Vector3 positionBias)
    {
        // Each row projects the relative velocity at the anchors (B - A) onto a world axis.
        for (var i = 0; i < Count; i++)
        {
            var axis = i switch { 0 => Vector3.UnitX, 1 => Vector3.UnitY, _ => Vector3.UnitZ };
            var angularJacobianA = Vector3.Cross(axis, offsetA);
            var angularJacobianB = -Vector3.Cross(axis, offsetB);
            var axisBias = Vector3.Dot(axis, positionBias);
            system.SetRow(i, axis, angularJacobianA, angularJacobianB, axisBias);
        }
    }
}
