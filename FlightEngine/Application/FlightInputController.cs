using System.Numerics;
using FlightEngine.Core;
using FlightEngine.Forces;
using FlightEngine.Physics;
using ImGuiNET;
using Raylib_cs;

namespace FlightEngine.Application;

public class FlightInputController : Component
{
    private RigidBody _body = null!;

    public override void OnInitialize()
    {
        _body = GameObject.GetComponent<RigidBody>() ??
                throw new InvalidOperationException("FlightInputController requires a RigidBody component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        if (ImGui.GetIO().WantCaptureKeyboard) return;

        HandleTranslationInput();
        HandleRotationInput();
    }

    private void HandleTranslationInput()
    {
        var transform = _body.Transform;

        var forceDirection = Vector3.Zero;
        if (Raylib.IsKeyDown(KeyboardKey.W)) forceDirection += transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.S)) forceDirection -= transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.A)) forceDirection -= transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.D)) forceDirection += transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.Space)) forceDirection += transform.LocalUp;

        const float forceMagnitude = 10.0f;
        var force = new ThrustForce(forceDirection, forceMagnitude);
        _body.AddForce(force);
    }

    private void HandleRotationInput()
    {
        var localApplicationPoint = Vector3.Zero;
        var forceMagnitude = 0.0f;
        var transform = _body.Transform;

        if (Raylib.IsKeyDown(KeyboardKey.Q))
        {
            localApplicationPoint += -transform.LocalRight * 2.0f;
            forceMagnitude = 5.0f;
        }

        if (Raylib.IsKeyDown(KeyboardKey.E))
        {
            localApplicationPoint += transform.LocalRight * 2.0f;
            forceMagnitude = 5.0f;
        }

        var rotationForce = new ThrustForce(-transform.LocalForward, forceMagnitude, localApplicationPoint);
        _body.AddForce(rotationForce);
    }
}