namespace OpenParol.Studio.Kinematics.Inverse;

/// <summary>
/// Defines the numerical parameters used by an inverse-kinematics solver.
/// </summary>
public sealed record InverseKinematicsOptions
{
    public int MaximumIterations { get; init; } = 100;

    public double PositionTolerance { get; init; } = 0.0001;

    public double OrientationTolerance { get; init; } =
        Math.PI / 1800.0;

    public double Damping { get; init; } = 0.01;

    public double MaximumJointStep { get; init; } =
        5.0 * Math.PI / 180.0;

    public double PositionWeight { get; init; } = 1.0;

    public double OrientationWeight { get; init; } = 0.25;

    public void Validate()
    {
        if (MaximumIterations <= 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(MaximumIterations),
                MaximumIterations,
                "The maximum iteration count must be positive."
            );
        }

        ValidatePositiveFinite(
            PositionTolerance,
            nameof(PositionTolerance)
        );

        ValidatePositiveFinite(
            OrientationTolerance,
            nameof(OrientationTolerance)
        );

        ValidatePositiveFinite(
            Damping,
            nameof(Damping)
        );

        ValidatePositiveFinite(
            MaximumJointStep,
            nameof(MaximumJointStep)
        );

        ValidatePositiveFinite(
            PositionWeight,
            nameof(PositionWeight)
        );

        ValidatePositiveFinite(
            OrientationWeight,
            nameof(OrientationWeight)
        );
    }

    private static void ValidatePositiveFinite(
        double value,
        string parameterName)
    {
        if (!double.IsFinite(value) || value <= 0.0)
        {
            throw new ArgumentOutOfRangeException(
                parameterName,
                value,
                "The value must be finite and greater than zero."
            );
        }
    }
}
