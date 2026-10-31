using FlightEngine.Physics;
using FlightEngine.Rendering;
using FlightEngine.Rendering.Debugging;
using FlightEngine.Rendering.Debugging.RigidBody;

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
        var airplane = flightScene.Airplane.Fuselage;
        var planeBody = airplane.GetComponent<RigidBody>()!;
        var camera = flightScene.Camera.GetComponent<Camera>()!;

        List<DebugSection> debugSections =
        [
            new PerformanceDebugSection(),
            new PositionDebugSection(airplane.Transform),
            new OrientationDebugSection(airplane.Transform),
            new RigidBodyDebugSection(planeBody),
            new CameraDebugSection(camera)
        ];
        return new DebugPanel(debugSections, appName: AppInfo.Name);
    }
}