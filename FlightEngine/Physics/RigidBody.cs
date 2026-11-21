using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Numerics;
using FlightEngine.Physics.Motion;

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

    public float Mass
    {
        get => _massProperties.Mass;
        set => _massProperties.Mass = value;
    }

    public float InverseMass
    {
        get => _massProperties.InverseMass;
        set => _massProperties.InverseMass = value;
    }

    public float MomentOfInertia
    {
        get => _massProperties.MomentOfInertia;
        set => _massProperties.MomentOfInertia = value;
    }

    public float InverseMomentOfInertia
    {
        get => _massProperties.InverseMomentOfInertia;
        set => _massProperties.InverseMomentOfInertia = value;
    }

    public bool HasInfiniteMass => _massProperties.HasInfiniteMass;

    public Vector3 CenterOfMass => _massProperties.CenterOfMass;

    public void IntegrateVelocities(float deltaTime)
    {
        if (HasInfiniteMass)
        {
            StopMotion();
            return;
        }

        _linearMotion.IntegrateVelocity(deltaTime);
        _angularMotion.IntegrateVelocity(deltaTime);
    }

    public void IntegrateTransform(float deltaTime)
    {
        if (HasInfiniteMass)
        {
            StopMotion();
            return;
        }

        _linearMotion.IntegrateTransform(Transform, deltaTime);
        _angularMotion.IntegrateTransform(Transform, deltaTime);
    }

    public void ClearAccumulators()
    {
        _linearMotion.ClearAccumulators();
        _angularMotion.ClearAccumulators();
    }

    public void StopMotion()
    {
        _linearMotion.StopMotion();
        _angularMotion.StopMotion();
    }

    public void ApplyForce(Vector3 worldForce, Vector3 localApplicationPoint)
    {
        _linearMotion.ApplyForce(worldForce);
        _angularMotion.ApplyTorque(worldForce, localApplicationPoint, Transform);
    }

    public void ApplyImpulse(Vector3 worldImpulse, Vector3 localApplicationPoint)
    {
        if (!worldImpulse.HasFiniteComponents() || !localApplicationPoint.HasFiniteComponents())
            return;

        _linearMotion.ApplyImpulse(worldImpulse);
        _angularMotion.ApplyImpulse(worldImpulse, localApplicationPoint, Transform);
    }

    public void ApplyAngularImpulse(Vector3 worldAngularImpulse)
    {
        if (worldAngularImpulse.HasFiniteComponents())
            _angularMotion.ApplyImpulse(worldAngularImpulse);
    }

    public Vector3 GetVelocityAtPoint(Vector3 localPoint)
    {
        var displacement = Transform.TransformVector(localPoint - CenterOfMass);
        var angularVelocityAtPoint = Vector3.Cross(AngularVelocity, displacement);
        return Velocity + angularVelocityAtPoint;
    }
}
