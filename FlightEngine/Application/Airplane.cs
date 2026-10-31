using FlightEngine.Core;

namespace FlightEngine.Application;

public class Airplane(GameObject leftWing, GameObject rightWing) : Component
{
    public GameObject Fuselage => GameObject;
    public GameObject LeftWing { get; } = leftWing;
    public GameObject RightWing { get; } = rightWing;
}
