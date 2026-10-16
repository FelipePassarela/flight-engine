namespace FlightEngine.Physics;

public class PhysicsWorld(IEnumerable<RigidBody> bodies)
{
    public void Step(float deltaTime)
    {
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