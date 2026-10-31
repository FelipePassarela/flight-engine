using System.Numerics;
using FlightEngine.Core;
using ImGuiNET;
using Raylib_cs;

namespace FlightEngine.Application;

public class FlightInputController : Component
{
    private Airplane _airplane = null!;

    public override void OnInitialize()
    {
        _airplane = GameObject.GetComponent<Airplane>() ??
                    throw new InvalidOperationException("FlightInputController requires an Airplane component.");
    }

    public override void OnUpdate(float deltaTime)
    {
        if (ImGui.GetIO().WantCaptureKeyboard) return;

        HandleTranslationInput();
        HandleRotationInput();
    }

    private void HandleTranslationInput()
    {
        var transform = _airplane.Fuselage.Transform;

        var forceDirection = Vector3.Zero;
        if (Raylib.IsKeyDown(KeyboardKey.W)) forceDirection += transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.S)) forceDirection -= transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.A)) forceDirection -= transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.D)) forceDirection += transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.Space)) forceDirection += transform.LocalUp;

        _airplane.ApplyThrust(forceDirection);
    }

    private void HandleRotationInput()
    {
        var rollInput = 0f;

        if (Raylib.IsKeyDown(KeyboardKey.Q))
            rollInput += 1f;
        if (Raylib.IsKeyDown(KeyboardKey.E))
            rollInput -= 1f;

        _airplane.ApplyRoll(rollInput);
    }
}
