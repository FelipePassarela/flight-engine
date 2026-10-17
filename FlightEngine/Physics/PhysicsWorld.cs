using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class PhysicsWorld(IEnumerable<RigidBody> bodies, IEnumerable<ForceApplier> forceAppliers)
{
    public void Step(float deltaTime)
    {
        foreach (var forceApplier in forceAppliers)
            forceApplier.ApplyForces();

        foreach (var body in bodies)
            body.IntegrateVelocities(deltaTime);

        HandleConstraints();

        foreach (var body in bodies)
        {
            body.IntegrateTransform(deltaTime);
            body.ClearAccumulators();
        }
    }

    private void HandleConstraints()
    {
        // not implemented yet
    }
}