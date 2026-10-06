using FlightEngine.Rendering;
using FlightEngine.Rendering.Debugging;

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

        List<DebugSection> debugSections =
        [
            new PerformanceDebugSection(),
            new PositionDebugSection(plane.Transform),
            new OrientationDebugSection(plane.Transform),
            new RigidBodyDebugSection(planeBody),
            new CameraDebugSection(camera)
        ];
        return new DebugPanel(debugSections, appName: AppInfo.Name);
    }
}