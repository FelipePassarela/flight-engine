using FlightEngine.Rendering;

namespace FlightEngine.Application;

public static class FlightRendererFactory
{
    public static RaylibSceneRenderer CreateSceneRenderer(FlightScene flightScene)
    {
        var debugPanel = CreateDebugPanel(flightScene);
        var camera = flightScene.Camera.GetComponent<Camera>()!;
        return new RaylibSceneRenderer(flightScene.Scene, camera, debugPanel, AppInfo.Name);
    }

    private static DebugPanel CreateDebugPanel(FlightScene flightScene)
    {
        var plane = flightScene.Plane;
        var planeBody = plane.GetComponent<RigidBody>()!;
        var camera = flightScene.Camera.GetComponent<Camera>()!;
        return new DebugPanel(plane.Transform, planeBody, camera, appName: AppInfo.Name);
    }
}