using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics;

public class Gravity(Vector3 gravityAcceleration = default, bool active = true) : Component
{
    public static Vector3 MoonGravityAcceleration => new(0.0f, -1.62f, 0.0f);
    public static Vector3 EarthGravityAcceleration => new(0.0f, -9.81f, 0.0f);

    public Vector3 GravityAcceleration { get; set; } = gravityAcceleration;
    public bool Active { get; set; } = active;
    private ForceApplier _forceApplier = null!;

    public override void OnInitialize()
    {
        _forceApplier = GameObject.GetComponent<ForceApplier>() ??
                        throw new InvalidOperationException("Gravity component requires a ForceApplier component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        if (Active) _forceApplier.AddForce(new GravityForce(GravityAcceleration));
    }
}