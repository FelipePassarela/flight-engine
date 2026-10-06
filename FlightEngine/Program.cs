using FlightEngine;
using FlightEngine.Application;

var flightScene = new FlightScene();
var sceneRenderer = FlightRendererFactory.CreateSceneRenderer(flightScene);
var runner = new EngineRunner(flightScene.Scene, sceneRenderer);

runner.Run();