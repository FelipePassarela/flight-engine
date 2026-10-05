using System.Diagnostics.CodeAnalysis;
using FlightEngine.Core;
using Raylib_cs;

namespace FlightEngine;

[method: SetsRequiredMembers]
public class Scene(Camera3D camera, List<GameObject> gameObjects)
{
    public required Camera3D Camera = camera;
    public required List<GameObject> GameObjects = gameObjects;
}