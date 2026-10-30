using FlightEngine.Core;

namespace FlightEngine.Physics.Forces;

public class ForceApplier : Component
{
    private readonly List<ForceGenerator> _forces = [];
    private RigidBody _body = null!;

    public override void OnInitialize()
    {
        _body = GameObject.GetComponent<RigidBody>() ??
                throw new InvalidOperationException("ForceApplier requires a RigidBody component.");
    }

    public void ApplyForces()
    {
        foreach (var generator in _forces)
        {
            var force = generator.CalculateWorldForce();
            _body.AddForce(force, generator.LocalApplicationPoint);
        }

        _forces.Clear();
    }

    public void AddForce(ForceGenerator force)
    {
        _forces.Add(force);
    }

    public ForceGenerator? GetForceGenerator<T>() where T : ForceGenerator
    {
        return _forces.OfType<T>().FirstOrDefault();
    }
}