using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Physics;
using FlightEngine.Physics.Forces;

namespace FlightEngine.Application;

public class Airplane(GameObject leftWing, GameObject rightWing) : Component
{
    private RigidBody _fuselageBody = null!;
    private RigidBody _leftWingBody = null!;
    private RigidBody _rightWingBody = null!;

    public GameObject Fuselage => GameObject;
    public GameObject LeftWing { get; } = leftWing;
    public GameObject RightWing { get; } = rightWing;

    public override void OnInitialize()
    {
        _fuselageBody = Fuselage.GetComponent<RigidBody>() ??
                        throw new InvalidOperationException(
                            "Airplane requires a RigidBody component on its fuselage.");
        _leftWingBody = LeftWing.GetComponent<RigidBody>() ??
                        throw new InvalidOperationException(
                            "Airplane requires a RigidBody component on its left wing.");
        _rightWingBody = RightWing.GetComponent<RigidBody>() ??
                         throw new InvalidOperationException(
                             "Airplane requires a RigidBody component on its right wing.");
    }

    public void ApplyThrust(Vector3 localDirection)
    {
        const float forceMagnitude = 10.0f;
        var force = new ThrustForce(_fuselageBody, localDirection, forceMagnitude, _fuselageBody.CenterOfMass);
        _fuselageBody.ApplyForce(force.CalculateWorldForce(), force.LocalApplicationPoint);
    }

    public void ApplyRoll(float rollInput)
    {
        var clampedInput = Math.Clamp(rollInput, -1f, 1f);
        const float forcePerWing = 2.5f;
        var worldForce = Fuselage.Transform.WorldUp * clampedInput * forcePerWing;

        // Equal and opposite forces create a torque without accelerating the center of mass.
        _leftWingBody.ApplyForce(worldForce, _leftWingBody.CenterOfMass);
        _rightWingBody.ApplyForce(-worldForce, _rightWingBody.CenterOfMass);
    }
}
