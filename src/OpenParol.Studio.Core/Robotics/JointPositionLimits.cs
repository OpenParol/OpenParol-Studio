namespace OpenParol.Studio.Core.Robotics;

/// <summary>
/// Represents the minimum and maximum allowed position of a revolute joint.
/// Values are expressed in radians.
/// </summary>
public readonly record struct JointPositionLimits
{
    public double Minimum { get; }

    public double Maximum { get; }

    public double Range => Maximum - Minimum;

    public JointPositionLimits(
        double minimum,
        double maximum)
    {
        if (!double.IsFinite(minimum))
        {
            throw new ArgumentOutOfRangeException(
                nameof(minimum),
                minimum,
                "The minimum joint position must be finite."
            );
        }

        if (!double.IsFinite(maximum))
        {
            throw new ArgumentOutOfRangeException(
                nameof(maximum),
                maximum,
                "The maximum joint position must be finite."
            );
        }

        if (minimum > maximum)
        {
            throw new ArgumentException(
                "The minimum joint position cannot be greater "
                + "than the maximum joint position.",
                nameof(minimum)
            );
        }

        Minimum = minimum;
        Maximum = maximum;
    }

    public bool Contains(double position)
    {
        return position >= Minimum
            && position <= Maximum;
    }

    public double Clamp(double position)
    {
        if (double.IsNaN(position))
        {
            throw new ArgumentOutOfRangeException(
                nameof(position),
                position,
                "The joint position cannot be NaN."
            );
        }

        return Math.Clamp(
            position,
            Minimum,
            Maximum
        );
    }
}
