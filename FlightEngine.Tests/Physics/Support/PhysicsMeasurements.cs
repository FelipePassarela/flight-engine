using System.Numerics;

namespace FlightEngine.Physics.Tests.Physics.Support;

internal static class PhysicsMeasurements
{
    public static Vector3 CalculateLinearMomentum(params RigidBody[] bodies) =>
        bodies.Aggregate(Vector3.Zero, AccumulateLinearMomentum);

    private static Vector3 AccumulateLinearMomentum(Vector3 sum, RigidBody body)
    {
        return sum + body.Mass * body.Velocity;
    }

    // Angular momentum about the world origin, including orbital and spin contributions.
    public static Vector3 CalculateAngularMomentum(params RigidBody[] bodies) =>
        bodies.Aggregate(Vector3.Zero, AccumulateAngularMomentum);

    private static Vector3 AccumulateAngularMomentum(Vector3 sum, RigidBody body) =>
        sum +
        Vector3.Cross(body.Transform.Position, body.Mass * body.Velocity) +
        body.MomentOfInertia * body.AngularVelocity;

    public static Vector3 CalculateCenterOfMass(params RigidBody[] bodies)
    {
        var totalMass = bodies.Sum(body => body.Mass);
        return bodies.Aggregate(Vector3.Zero, AccumulateMassWeightedPosition) / totalMass;
    }

    private static Vector3 AccumulateMassWeightedPosition(Vector3 sum, RigidBody body) =>
        sum + body.Mass * body.Transform.Position;
}