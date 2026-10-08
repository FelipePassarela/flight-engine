using System.Numerics;

namespace FlightEngine.Physics;

public class MassProperties
{
    public MassProperties(float mass, float momentOfInertia, Vector3 centerOfMass)
    {
        Mass = mass;
        MomentOfInertia = momentOfInertia;
        CenterOfMass = centerOfMass;
    }

    public float Mass
    {
        get;
        set
        {
            ArgumentOutOfRangeException.ThrowIfNegative(value);
            if (!float.IsFinite(value))
                throw new ArgumentOutOfRangeException(nameof(value), "Mass must be finite.");

            field = value;
        }
    }

    public float InverseMass
    {
        get => Mass > 0.0f ? 1.0f / Mass : 0.0f;
        set
        {
            ArgumentOutOfRangeException.ThrowIfNegative(value);
            if (!float.IsFinite(value))
                throw new ArgumentOutOfRangeException(nameof(value), "Inverse mass must be finite.");

            Mass = value > 0.0f ? 1.0f / value : 0.0f;
        }
    }

    // Zero represents infinite mass.
    public bool HasInfiniteMass => Mass == 0.0f;

    public float MomentOfInertia // TODO: Change moment of inertia to tensor of inertia for complex shape
    {
        get;
        set
        {
            ArgumentOutOfRangeException.ThrowIfNegative(value);
            if (!float.IsFinite(value))
                throw new ArgumentOutOfRangeException(nameof(value), "Moment of inertia must be finite.");

            field = value;
        }
    }

    public Vector3 CenterOfMass { get; set; }
}