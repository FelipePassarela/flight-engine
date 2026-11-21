namespace FlightEngine.Physics.Constraints;

/// <summary>
/// Defines an interval of signed hinge angles relative to the initial pose.
/// Both angles are in radians and must be strictly between -pi and pi.
/// </summary>
public sealed class HingeAngleLimits
{
    public HingeAngleLimits(float minimumRadians, float maximumRadians)
    {
        ValidateAngle(minimumRadians, nameof(minimumRadians));
        ValidateAngle(maximumRadians, nameof(maximumRadians));
        if (maximumRadians <= minimumRadians)
            throw new ArgumentOutOfRangeException(nameof(maximumRadians), maximumRadians,
                "The maximum angle must be greater than the minimum angle.");

        MinimumRadians = minimumRadians;
        MaximumRadians = maximumRadians;
    }

    public float MinimumRadians { get; }
    public float MaximumRadians { get; }

    private static void ValidateAngle(float angle, string parameterName)
    {
        if (!float.IsFinite(angle) || angle <= -MathF.PI || angle >= MathF.PI)
            throw new ArgumentOutOfRangeException(parameterName, angle,
                "Hinge angles must be finite and strictly between -pi and pi.");
    }
}
