using FlightEngine.Core;
using FlightEngine.Physics;

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
        foreach (var generator in forces)
        {
            var worldForce = generator.CalculateWorldForce(_body);
            _body.AddForce(worldForce, generator.LocalApplicationPoint);
        }
    }
}