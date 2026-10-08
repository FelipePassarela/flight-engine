using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Forces;

namespace FlightEngine.Physics;

public class RigidBody(float mass = 1.0f, float momentOfInertia = 1.0f) : Component
{
    public Transform Transform => GameObject.Transform;
    public Vector3 Velocity { get; set; } = Vector3.Zero;
    private Vector3 _accumulatedForce = Vector3.Zero;

    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;
    private Vector3 _accumulatedTorque = Vector3.Zero;

    private readonly MassProperties _massProperties = new(mass, momentOfInertia, Vector3.Zero);

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

    public bool UseGravity { get; set; } = true;

    public override void OnUpdate(float deltaTime)
    {
        if (HasInfiniteMass)
        {
            StopMotion();
            return;
        }

        IntegrateLinearMotion(deltaTime);
        IntegrateAngularMotion(deltaTime);
    }

    public void StopMotion()
    {
        _accumulatedForce = Vector3.Zero;
        _accumulatedTorque = Vector3.Zero;
        Velocity = Vector3.Zero;
        AngularVelocity = Vector3.Zero;
    }

    private void IntegrateLinearMotion(float deltaTime)
    {
        var acceleration = _accumulatedForce * InverseMass;
        Velocity += acceleration * deltaTime;
        Transform.Position += Velocity * deltaTime;
        _accumulatedForce = Vector3.Zero;
    }

    private void IntegrateAngularMotion(float deltaTime)
    {
        var angularAcceleration = _accumulatedTorque / MomentOfInertia;
        AngularVelocity += angularAcceleration * deltaTime;
        _accumulatedTorque = Vector3.Zero;

        var angularSpeed = AngularVelocity.Length();
        if (angularSpeed <= 0.0f) return;

        var rotationAxis = AngularVelocity / angularSpeed;
        var rotationAngle = angularSpeed * deltaTime;
        var deltaRotation = Quaternion.CreateFromAxisAngle(rotationAxis, rotationAngle);
        Transform.Orientation = Quaternion.Normalize(deltaRotation * Transform.Orientation);
    }

    public void AddForce(ForceGenerator generator)
    {
        if (!UseGravity && generator is GravityForce)
            return;

        var worldForce = generator.CalculateWorldForce(this);
        _accumulatedForce += worldForce;

        var localOffset = generator.LocalApplicationPoint - CenterOfMass;
        var worldOffset = Vector3.Transform(localOffset, Transform.Orientation);
        _accumulatedTorque += Vector3.Cross(worldOffset, worldForce);
    }

    public void AddForces(IEnumerable<ForceGenerator> forces)
    {
        foreach (var force in forces)
        {
            AddForce(force);
        }
    }
}