using System.Numerics;
using FlightEngine.Forces;

namespace FlightEngine;

public class RigidBody(Transform transform, float mass = 1.0f, float momentOfInertia = 1.0f)
{
    public Transform Transform { get; } = transform;
    public Vector3 Velocity { get; set; } = Vector3.Zero;
    private Vector3 _accumulatedForce = Vector3.Zero;

    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;
    public float MomentOfInertia { get; set; } = momentOfInertia; // TODO: Change to tensor of inertia for complex shape
    private Vector3 _accumulatedTorque = Vector3.Zero;

    public float InverseMass
    {
        get;
        set => field = value > 0.0f ? value : 0.0f;
    } = mass > 0.0f ? 1.0f / mass : 0.0f;

    public float Mass => 1.0f / InverseMass; // For convenience
    public bool HasInfiniteMass => InverseMass <= 0.0f;
    public Vector3 CenterOfMass { get; } = Vector3.Zero;


    public void Integrate(float deltaTime)
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

        var angularSpeed = AngularVelocity.Length();
        if (angularSpeed <= 0.0f) return;

        var rotationAxis = AngularVelocity / angularSpeed;
        var rotationAngle = angularSpeed * deltaTime;
        var deltaRotation = Quaternion.CreateFromAxisAngle(rotationAxis, rotationAngle);
        Transform.Orientation = Quaternion.Normalize(deltaRotation * Transform.Orientation);

        _accumulatedTorque = Vector3.Zero;
    }

    public void AddForce(ForceGenerator generator)
    {
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