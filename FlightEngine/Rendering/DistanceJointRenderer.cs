using FlightEngine.Physics.Constraints;
using Raylib_cs;

namespace FlightEngine.Rendering;

internal sealed class DistanceJointRenderer : Renderer
{
    private DistanceJoint _joint = null!;

    public override void OnInitialize()
    {
        _joint = GameObject.GetComponent<DistanceJoint>() ??
                 throw new InvalidOperationException("DistanceJointRenderer requires a DistanceJoint component.");
    }

    public override void Render()
    {
        var anchorA = _joint.BodyA.Transform.TransformPoint(_joint.LocalAnchorA);
        var anchorB = _joint.BodyB.Transform.TransformPoint(_joint.LocalAnchorB);
        var color = Color.DarkGreen;
        const float rodRadius = 0.035f;
        const float anchorRadius = 0.09f;

        if (anchorA != anchorB)
            Raylib.DrawCylinderEx(anchorA, anchorB, rodRadius, rodRadius, 8, color);

        Raylib.DrawSphere(anchorA, anchorRadius, color);
        Raylib.DrawSphere(anchorB, anchorRadius, color);
    }
}
