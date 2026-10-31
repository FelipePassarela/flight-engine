using System.Numerics;

namespace FlightEngine.Core;

public class Transform : Component
{
    public Vector3 Position { get; set; }
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector3 Scale { get; set; } = Vector3.One;

    public readonly Vector3 LocalForward = Vector3.UnitZ;
    public readonly Vector3 LocalUp = Vector3.UnitY;
    public readonly Vector3 LocalRight = -Vector3.UnitX;
    public readonly Vector3 LocalLeft = Vector3.UnitX;

    public Vector3 WorldForward => TransformDirection(LocalForward);
    public Vector3 WorldUp => TransformDirection(LocalUp);
    public Vector3 WorldRight => TransformDirection(LocalRight);
    public Vector3 WorldLeft => -WorldRight;

    public void LookAt(Vector3 targetPosition, Vector3 up)
    {
        var direction = Vector3.Normalize(targetPosition - Position);
        var right = Vector3.Normalize(Vector3.Cross(up, direction));
        var correctedUp = Vector3.Cross(direction, right);

        var rotationMatrix = new Matrix4x4(
            right.X, right.Y, right.Z, 0,
            correctedUp.X, correctedUp.Y, correctedUp.Z, 0,
            direction.X, direction.Y, direction.Z, 0,
            0, 0, 0, 1
        );

        Orientation = Quaternion.CreateFromRotationMatrix(rotationMatrix);
    }

    public Vector3 TransformPoint(Vector3 point)
    {
        var scaledPoint = point * Scale;
        var rotatedPoint = Vector3.Transform(scaledPoint, Orientation);
        return Position + rotatedPoint;
    }

    public Vector3 InverseTransformPoint(Vector3 point)
    {
        var translatedPoint = point - Position;
        var rotatedPoint = Vector3.Transform(translatedPoint, Quaternion.Inverse(Orientation));
        return rotatedPoint / Scale;
    }

    public Vector3 TransformDirection(Vector3 direction) => Vector3.Transform(direction, Orientation);

    public Vector3 InverseTransformDirection(Vector3 direction) =>
        Vector3.Transform(direction, Quaternion.Inverse(Orientation));
}