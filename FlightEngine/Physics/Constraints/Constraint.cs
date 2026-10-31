using FlightEngine.Core;

namespace FlightEngine.Physics.Constraints;

public abstract class Constraint : Component
{
    public abstract void Prepare(float deltaTime);
    
    public abstract void SolveVelocity(float deltaTime);
}