using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Inverse;

/// <summary>
/// Represents the Cartesian position and orientation error between
/// two robot poses.
/// </summary>
public readonly record struct PoseError(
    Vector3D Position,
    Vector3D Orientation)
{
    public double PositionMagnitude =>
        Position.Length;

    public double OrientationMagnitude =>
        Orientation.Length;

    public static PoseError Calculate(
        RobotPose current,
        RobotPose target)
    {
        var positionError =
            target.Position - current.Position;

        var currentOrientation =
            current.Orientation.Normalize();

        var targetOrientation =
            target.Orientation.Normalize();

        var relativeOrientation =
            (
                targetOrientation
                * currentOrientation.Conjugate()
            ).Normalize();

        if (relativeOrientation.W < 0.0)
        {
            relativeOrientation = new QuaternionD(
                -relativeOrientation.X,
                -relativeOrientation.Y,
                -relativeOrientation.Z,
                -relativeOrientation.W
            );
        }

        var vectorPart = new Vector3D(
            relativeOrientation.X,
            relativeOrientation.Y,
            relativeOrientation.Z
        );

        var orientationError =
            CalculateRotationVector(
                vectorPart,
                relativeOrientation.W
            );

        return new(
            positionError,
            orientationError
        );
    }

    private static Vector3D CalculateRotationVector(
        Vector3D vectorPart,
        double scalarPart)
    {
        if (vectorPart.LengthSquared == 0.0)
        {
            return Vector3D.Zero;
        }

        var vectorLength =
            vectorPart.Length;

        var angle =
            2.0 * Math.Atan2(
                vectorLength,
                scalarPart
            );

        var axis =
            vectorPart * (1.0 / vectorLength);

        return axis * angle;
    }
}
