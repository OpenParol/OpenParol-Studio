namespace OpenParol.Studio.Core.Geometry;

/// <summary>
/// Represents a quaternion using double-precision values.
/// </summary>
/// <remarks>
/// The quaternion uses the Hamilton convention. X, Y and Z form the
/// vector component, while W is the scalar component.
/// </remarks>
public readonly record struct QuaternionD(
    double X,
    double Y,
    double Z,
    double W)
{
    public static QuaternionD Identity { get; } =
        new(0.0, 0.0, 0.0, 1.0);

    public double Length => Math.Sqrt(LengthSquared);

    public double LengthSquared =>
        (X * X) + (Y * Y) + (Z * Z) + (W * W);

    public QuaternionD Normalize()
    {
        if (LengthSquared == 0.0)
        {
            throw new InvalidOperationException(
                "A zero-length quaternion cannot be normalized.");
        }

        var inverseLength = 1.0 / Length;

        return new(
            X * inverseLength,
            Y * inverseLength,
            Z * inverseLength,
            W * inverseLength
        );
    }

    public QuaternionD Conjugate()
    {
        return new(
            -X,
            -Y,
            -Z,
            W
        );
    }

    public Vector3D Rotate(Vector3D vector)
    {
        var normalized = Normalize();

        var vectorQuaternion = new QuaternionD(
            vector.X,
            vector.Y,
            vector.Z,
            0.0
        );

        var rotated =
            normalized
            * vectorQuaternion
            * normalized.Conjugate();

        return new(
            rotated.X,
            rotated.Y,
            rotated.Z
        );
    }

    public static QuaternionD FromRollPitchYaw(
        double roll,
        double pitch,
        double yaw)
    {
        if (!double.IsFinite(roll))
        {
            throw new ArgumentOutOfRangeException(
                nameof(roll),
                roll,
                "The roll angle must be finite."
            );
        }

        if (!double.IsFinite(pitch))
        {
            throw new ArgumentOutOfRangeException(
                nameof(pitch),
                pitch,
                "The pitch angle must be finite."
            );
        }

        if (!double.IsFinite(yaw))
        {
            throw new ArgumentOutOfRangeException(
                nameof(yaw),
                yaw,
                "The yaw angle must be finite."
            );
        }

        var rollRotation = FromAxisAngle(
            new Vector3D(1.0, 0.0, 0.0),
            roll
        );

        var pitchRotation = FromAxisAngle(
            new Vector3D(0.0, 1.0, 0.0),
            pitch
        );

        var yawRotation = FromAxisAngle(
            new Vector3D(0.0, 0.0, 1.0),
            yaw
        );

        return (
            yawRotation
            * pitchRotation
            * rollRotation
        ).Normalize();
    }


    public static QuaternionD FromAxisAngle(
        Vector3D axis,
        double angle)
    {
        if (axis.LengthSquared == 0.0)
        {
            throw new ArgumentException(
                "The rotation axis cannot be a zero-length vector.",
                nameof(axis));
        }

        var normalizedAxis = axis.Normalize();
        var halfAngle = angle / 2.0;
        var sinHalfAngle = Math.Sin(halfAngle);
        var cosHalfAngle = Math.Cos(halfAngle);

        return new(
            normalizedAxis.X * sinHalfAngle,
            normalizedAxis.Y * sinHalfAngle,
            normalizedAxis.Z * sinHalfAngle,
            cosHalfAngle
        );
    }

    public static QuaternionD operator *(
        QuaternionD left,
        QuaternionD right)
    {
        return new(
            (left.W * right.X)
                + (left.X * right.W)
                + (left.Y * right.Z)
                - (left.Z * right.Y),

            (left.W * right.Y)
                - (left.X * right.Z)
                + (left.Y * right.W)
                + (left.Z * right.X),

            (left.W * right.Z)
                + (left.X * right.Y)
                - (left.Y * right.X)
                + (left.Z * right.W),

            (left.W * right.W)
                - (left.X * right.X)
                - (left.Y * right.Y)
                - (left.Z * right.Z)
        );
    }
}
