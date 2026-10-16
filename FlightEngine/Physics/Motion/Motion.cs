using FlightEngine.Core;

namespace FlightEngine.Physics.Motion;

public abstract class Motion
{
    public void Integrate(Transform transform, float deltaTime)
    {
        IntegrateVelocity(deltaTime);
        IntegrateTransform(transform, deltaTime);
        ClearAccumulators();
    }

    public abstract void StopMotion();

    public abstract void IntegrateVelocity(float deltaTime);

    public abstract void IntegrateTransform(Transform transform, float deltaTime);

    public abstract void ClearAccumulators();
}