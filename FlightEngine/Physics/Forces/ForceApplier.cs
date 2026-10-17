using System.Numerics;
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
            ApplyReactionForce(generator, force);
        }

        _forces.Clear();
    }

    private static void ApplyReactionForce(ForceGenerator generator, Vector3 force)
    {
        if (generator is not PairForceGenerator pairGenerator) return;
        var otherBody = pairGenerator.OtherBody;
        otherBody.AddForce(-force, pairGenerator.OtherLocalApplicationPoint);
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