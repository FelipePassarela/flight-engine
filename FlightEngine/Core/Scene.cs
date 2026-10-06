using System.Diagnostics.CodeAnalysis;

namespace FlightEngine.Core;

[method: SetsRequiredMembers]
public class Scene(List<GameObject> gameObjects)
{
    public required List<GameObject> GameObjects = gameObjects;
}