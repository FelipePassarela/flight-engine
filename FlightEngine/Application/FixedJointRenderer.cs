using FlightEngine.Physics.Constraints;
using FlightEngine.Rendering;
using Raylib_cs;

namespace FlightEngine.Application;

public class FixedJointRenderer : Renderer
{
    private FixedJoint _joint = null!;

    public override void OnInitialize()
    {
        _joint = GameObject.GetComponent<FixedJoint>() ??
                 throw new InvalidOperationException("FixedJointRenderer requires a FixedJoint component.");
    }

    public override void Render()
    {
        Raylib.DrawSphere(_joint.ContactPoint, 0.2f, Color.DarkBlue);
        var positionA = _joint.BodyA.Transform.Position;
        var positionB = _joint.BodyB.Transform.Position;
        Raylib.DrawLine3D(positionA, positionB, Color.DarkBlue);
    }
}