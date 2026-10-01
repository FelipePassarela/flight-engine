using System.Numerics;
using FlightEngine.Forces;
using ImGuiNET;
using Raylib_cs;

namespace FlightEngine;

public static class FlightInputController
{
    public static void Update(RigidBody body)
    {
        if (ImGui.GetIO().WantCaptureKeyboard) return;

        HandleTranslationInput(body);
        HandleRotationInput(body);
    }

    private static void HandleTranslationInput(RigidBody body)
    {
        var transform = body.Transform;

        var forceDirection = Vector3.Zero;
        if (Raylib.IsKeyDown(KeyboardKey.W)) forceDirection += transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.S)) forceDirection -= transform.LocalForward;
        if (Raylib.IsKeyDown(KeyboardKey.A)) forceDirection -= transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.D)) forceDirection += transform.LocalRight;
        if (Raylib.IsKeyDown(KeyboardKey.Space)) forceDirection += transform.LocalUp;

        const float forceMagnitude = 10.0f;
        var force = new ThrustForce(forceDirection, forceMagnitude);
        body.AddForce(force);
    }

    private static void HandleRotationInput(RigidBody body)
    {
        var localApplicationPoint = Vector3.Zero;
        var forceMagnitude = 0.0f;
        var transform = body.Transform;

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
        body.AddForce(rotationForce);
    }
}