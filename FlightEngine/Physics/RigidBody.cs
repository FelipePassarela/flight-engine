using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics;

public class RigidBody : Component
{
    public RigidBody(float mass = 1.0f, float momentOfInertia = 1.0f)
    {
        _massProperties = new MassProperties(mass, momentOfInertia, Vector3.Zero);
        _linearMotion = new LinearMotion(_massProperties);
        _angularMotion = new AngularMotion(_massProperties);
    }

    public Transform Transform => GameObject.Transform;
    private readonly AngularMotion _angularMotion;
    private readonly LinearMotion _linearMotion;
    private readonly MassProperties _massProperties;
    public bool UseGravity { get; set; } = true;

    public Vector3 AngularVelocity
    {
        get => _angularMotion.AngularVelocity;
        set => _angularMotion.AngularVelocity = value;
    }

    public Vector3 Velocity
    {
        get => _linearMotion.Velocity;
        set => _linearMotion.Velocity = value;
    }

    public float MomentOfInertia
    {
        get => _massProperties.MomentOfInertia;
        set => _massProperties.MomentOfInertia = value;
    }

    public float InverseMass
    {
        get => _massProperties.InverseMass;
        set => _massProperties.InverseMass = value;
    }

    public float Mass
    {
        get => _massProperties.Mass;
        set => _massProperties.Mass = value;
    }

    public bool HasInfiniteMass => _massProperties.HasInfiniteMass;

    public Vector3 CenterOfMass => _massProperties.CenterOfMass;

    public override void OnUpdate(float deltaTime)
    {
        if (HasInfiniteMass)
        {
            StopMotion();
            return;
        }

        _linearMotion.Integrate(Transform, deltaTime);
        _angularMotion.Integrate(Transform, deltaTime);
    }

    public void StopMotion()
    {
        _linearMotion.StopMotion();
        _angularMotion.StopMotion();
    }

    public void AddForce(Vector3 worldForce, Vector3 localApplicationPoint)
    {
        _linearMotion.AddForce(worldForce);
        _angularMotion.AddTorque(worldForce, localApplicationPoint, Transform.Orientation);
    }
}