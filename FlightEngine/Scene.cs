using System.Diagnostics.CodeAnalysis;
using FlightEngine.Core;
using FlightEngine.Rendering;

namespace FlightEngine;

[method: SetsRequiredMembers]
public class Scene(Camera activeCamera, List<GameObject> gameObjects)
{
    public required Camera ActiveCamera = activeCamera;
    public required List<GameObject> GameObjects = gameObjects;
}