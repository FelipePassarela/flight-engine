using System.Diagnostics.CodeAnalysis;

namespace FlightEngine.Core;

[method: SetsRequiredMembers]
public class Scene(List<GameObject> gameObjects)
{
    public IEnumerable<T> GetComponents<T>() where T : Component => gameObjects.SelectMany(go => go.GetComponents<T>());

    public IEnumerable<Component> GetComponents() => gameObjects.SelectMany(go => go.GetComponents());
}