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

    public Vector3 WorldForward => Vector3.Transform(LocalForward, Orientation);
    public Vector3 WorldUp => Vector3.Transform(LocalUp, Orientation);
    public Vector3 WorldRight => Vector3.Transform(LocalRight, Orientation);
}