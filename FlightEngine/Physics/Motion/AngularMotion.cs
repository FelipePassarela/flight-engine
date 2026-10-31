using System.Numerics;
using FlightEngine.Core;

namespace FlightEngine.Physics.Motion;

internal class AngularMotion(MassProperties massProperties) : Motion
{
    private Vector3 _accumulatedTorque = Vector3.Zero;
    public Vector3 AngularVelocity { get; set; } = Vector3.Zero;

    public void AddTorque(Vector3 worldForce, Vector3 localApplicationPoint, Transform bodyTransform)
    {
        var localOffset = localApplicationPoint - massProperties.CenterOfMass;
        var offset = bodyTransform.TransformVector(localOffset);
        _accumulatedTorque += Vector3.Cross(offset, worldForce);
    }

    public void AddImpulse(Vector3 worldImpulse, Vector3 localApplicationPoint, Transform bodyTransform)
    {
        var localOffset = localApplicationPoint - massProperties.CenterOfMass;
        var offset = bodyTransform.TransformVector(localOffset);
        var angularImpulse = Vector3.Cross(offset, worldImpulse);
        AngularVelocity += angularImpulse * massProperties.InverseMomentOfInertia;
    }

    public void AddImpulse(Vector3 worldAngularImpulse)
    {
        AngularVelocity += worldAngularImpulse * massProperties.InverseMomentOfInertia;
    }

    public override void IntegrateVelocity(float deltaTime)
    {
        var angularAcceleration = _accumulatedTorque * massProperties.InverseMomentOfInertia;
        AngularVelocity += angularAcceleration * deltaTime;
    }

    public override void IntegrateTransform(Transform transform, float deltaTime)
    {
        var angularSpeed = AngularVelocity.Length();
        if (angularSpeed <= 0.0f) return;

        var rotationAxis = AngularVelocity / angularSpeed;
        var rotationAngle = angularSpeed * deltaTime;
        var deltaRotation = Quaternion.CreateFromAxisAngle(rotationAxis, rotationAngle);
        transform.Orientation = Quaternion.Normalize(deltaRotation * transform.Orientation);
    }

    public override void ClearAccumulators()
    {
        _accumulatedTorque = Vector3.Zero;
    }

    public override void StopMotion()
    {
        ClearAccumulators();
        AngularVelocity = Vector3.Zero;
    }
}