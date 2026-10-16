using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics.Motion;

internal class AngularMotion(MassProperties massProperties) : IMotion
{
    private Vector3 _accumulatedTorque = Vector3.Zero;
    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;

    public void Integrate(Transform transform, float deltaTime)
    {
        IntegrateAngularVelocity(deltaTime);
        IntegrateOrientation(transform, deltaTime);
    }

    private void IntegrateAngularVelocity(float deltaTime)
    {
        var angularAcceleration = _accumulatedTorque / massProperties.MomentOfInertia;
        AngularVelocity += angularAcceleration * deltaTime;
        _accumulatedTorque = Vector3.Zero;
    }

    private void IntegrateOrientation(Transform transform, float deltaTime)
    {
        var angularSpeed = AngularVelocity.Length();
        if (angularSpeed <= 0.0f) return;

        var rotationAxis = AngularVelocity / angularSpeed;
        var rotationAngle = angularSpeed * deltaTime;
        var deltaRotation = Quaternion.CreateFromAxisAngle(rotationAxis, rotationAngle);
        transform.Orientation = Quaternion.Normalize(deltaRotation * transform.Orientation);
    }

    public void AddTorque(Vector3 worldForce, Vector3 localApplicationPoint, Quaternion bodyOrientation)
    {
        var localOffset = localApplicationPoint - massProperties.CenterOfMass;
        var worldOffset = Vector3.Transform(localOffset, bodyOrientation);
        _accumulatedTorque += Vector3.Cross(worldOffset, worldForce);
    }

    public void StopMotion()
    {
        _accumulatedTorque = Vector3.Zero;
        AngularVelocity = Vector3.Zero;
    }
}