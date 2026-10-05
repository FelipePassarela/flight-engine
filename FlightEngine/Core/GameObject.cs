namespace FlightEngine.Core;

public class GameObject
{
    public Transform Transform { get; }
    private readonly List<Component> _components;

    public GameObject() : this(new Transform())
    {
    }

    public GameObject(Transform transform)
    {
        Transform = transform;
        Transform.GameObject = this;
        _components = [Transform];
    }

    public GameObject(List<Component> components) : this(GetOrCreateTransform(components))
    {
        foreach (var component in components.Where(c => c is not Core.Transform))
        {
            AddComponent(component);
        }
    }

    private static Transform GetOrCreateTransform(List<Component> components)
    {
        var transformsPassed = components.OfType<Transform>().Count();
        return transformsPassed switch
        {
            > 1 => throw new ArgumentException("A GameObject can only have one Transform component."),
            1 => components.OfType<Transform>().First(),
            _ => new Transform()
        };
    }

    public T AddComponent<T>(T component) where T : Component
    {
        if (component.GameObject is not null)
        {
            throw new ArgumentException("Component already belongs to a GameObject.");
        }

        if (component is Transform)
        {
            throw new InvalidOperationException("A GameObject already has a Transform.");
        }

        _components.Add(component);
        component.GameObject = this;

        return component;
    }

    public T? GetComponent<T>() where T : Component => _components.OfType<T>().FirstOrDefault();

    public IEnumerable<T> GetComponents<T>() where T : Component => _components.OfType<T>();
}