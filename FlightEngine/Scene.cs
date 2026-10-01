using System.Diagnostics.CodeAnalysis;
using Raylib_cs;

namespace FlightEngine;

[method: SetsRequiredMembers]
public class Scene(Camera3D camera, Transform planeTransform, RigidBody planeBody)
{
    public required Camera3D Camera = camera;
    public required Transform PlaneTransform { get; init; } = planeTransform;
    public required RigidBody PlaneBody { get; init; } = planeBody;
}