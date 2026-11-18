using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Inverse;

/// <summary>
/// Contains the result of an inverse-kinematics calculation.
/// </summary>
public sealed record InverseKinematicsResult
{
    public InverseKinematicsStatus Status { get; }

    public JointVector JointPositions { get; }

    public int Iterations { get; }

    public double PositionError { get; }

    public double OrientationError { get; }

    public bool Converged =>
        Status == InverseKinematicsStatus.Converged;

    public InverseKinematicsResult(
        InverseKinematicsStatus status,
        JointVector jointPositions,
        int iterations,
        double positionError,
        double orientationError)
    {
        if (iterations < 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(iterations),
                iterations,
                "The iteration count cannot be negative."
            );
        }

        ValidateError(
            positionError,
            nameof(positionError)
        );

        ValidateError(
            orientationError,
            nameof(orientationError)
        );

        Status = status;
        JointPositions = jointPositions;
        Iterations = iterations;
        PositionError = positionError;
        OrientationError = orientationError;
    }

    private static void ValidateError(
        double value,
        string parameterName)
    {
        if (!double.IsFinite(value) || value < 0.0)
        {
            throw new ArgumentOutOfRangeException(
                parameterName,
                value,
                "The error value must be finite and non-negative."
            );
        }
    }
}
