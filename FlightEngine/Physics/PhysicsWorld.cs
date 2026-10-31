using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class PhysicsWorld(
    IEnumerable<RigidBody> bodies,
    IEnumerable<ForceApplier> forceAppliers,
    IEnumerable<Constraint> constraints,
    int constraintSolverIterations = 10)
{
    public void Step(float deltaTime)
    {
        foreach (var forceApplier in forceAppliers)
            forceApplier.ApplyForces();

        foreach (var body in bodies)
            body.IntegrateVelocities(deltaTime);

        HandleConstraints(deltaTime);

        foreach (var body in bodies)
        {
            body.IntegrateTransform(deltaTime);
            body.ClearAccumulators();
        }
    }

    private void HandleConstraints(float deltaTime)
    {
        foreach (var constraint in constraints)
            constraint.Prepare(deltaTime);

        for (var i = 0; i < constraintSolverIterations; i++)
            foreach (var constraint in constraints)
                constraint.SolveVelocity(deltaTime);
    }
}