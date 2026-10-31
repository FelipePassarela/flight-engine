using FlightEngine.Core;
using FlightEngine.Physics.Constraints;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Physics.Tests.Physics.Support;

// Runs the real component lifecycle and world without keyboard, GUI or rendering calls.
internal sealed class PhysicsSimulation
{
    private readonly PhysicsWorld _world;
    private readonly Component[] _physicsUpdates;

    public PhysicsSimulation(params GameObject[] gameObjects)
    {
        var scene = new Scene([.. gameObjects]);
        foreach (var component in scene.GetComponents())
            component.OnInitialize();

        _physicsUpdates = [.. scene.GetComponents().Where(c => c is Gravity or Spring)];
        _world = new PhysicsWorld(
            scene.GetComponents<RigidBody>(),
            scene.GetComponents<ForceApplier>(),
            scene.GetComponents<Constraint>());
    }

    public void RunFor(
        int durationSeconds,
        int stepsPerSecond,
        Action<int>? beforeStep = null,
        Action? afterStep = null)
    {
        var deltaTime = 1f / stepsPerSecond;
        for (var step = 0; step < durationSeconds * stepsPerSecond; step++)
        {
            beforeStep?.Invoke(step);
            Step(deltaTime);
            afterStep?.Invoke();
        }
    }

    public void Step(float deltaTime)
    {
        foreach (var component in _physicsUpdates)
            component.OnUpdate(deltaTime);
        _world.Step(deltaTime);
    }
}