using FlightEngine.Core;

namespace FlightEngine.Forces;

public class ForceApplier(IEnumerable<ForceGenerator> forces) : Component
{
    private RigidBody _body = null!;

    public override void OnInitialize()
    {
        _body = GameObject.GetComponent<RigidBody>() ??
                throw new InvalidOperationException("ForceApplier requires a RigidBody component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        _body.AddForces(forces);
    }
}