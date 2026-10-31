using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics.Tests.Physics.Support;

internal static class PhysicsObjects
{
    public static RigidBody Body(
        float mass = 1,
        float inertia = 1,
        Vector3 position = default,
        Quaternion? orientation = null,
        Vector3? gravity = null)
    {
        var gameObject = new GameObject([
            new Transform { Position = position, Orientation = orientation ?? Quaternion.Identity },
            new ForceApplier(),
            new RigidBody(mass, inertia)
        ]);

        if (gravity.HasValue)
            gameObject.AddComponent(new Gravity(gravity.Value));

        return gameObject.GetComponent<RigidBody>()!;
    }

    public static GameObject Joint(RigidBody a, RigidBody b, Vector3 worldAnchor) =>
        new([new FixedJoint(a, b, worldAnchor)]);
}