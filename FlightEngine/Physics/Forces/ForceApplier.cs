using FlightEngine.Core;

namespace FlightEngine.Physics.Forces;

public class ForceApplier(IEnumerable<ForceGenerator> forces, bool useGravity = true) : Component
{
    private RigidBody _body = null!;
    public bool UseGravity { get; set; } = useGravity;
    
    public override void OnInitialize()
    {
        _body = GameObject.GetComponent<RigidBody>() ??
                throw new InvalidOperationException("ForceApplier requires a RigidBody component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        foreach (var generator in forces)
        {
            if (generator is GravityForce && !UseGravity)
                continue;
            
            var worldForce = generator.CalculateWorldForce(_body);
            _body.AddForce(worldForce, generator.LocalApplicationPoint);
        }
    }
    
    public ForceGenerator? GetForceGenerator<T>() where T : ForceGenerator
    {
        return forces.OfType<T>().FirstOrDefault();
    }
}