namespace FlightEngine.Core;

public abstract class Component
{
    public GameObject GameObject { get; internal set; } = null!;

    public virtual void OnInitialize()
    {
    }

    public virtual void OnUpdate(float deltaTime)
    {
    }
}