using FlightEngine.Core;

namespace FlightEngine.Physics.Motion;

internal interface IMotion
{
    public void Integrate(Transform transform, float deltaTime);
    public void StopMotion();
}