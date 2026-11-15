namespace OpenParol.Studio.Core.Geometry;

/// <summary>
/// Represents a three-dimensional vector using double-precision values.
/// </summary>
public readonly record struct Vector3D(
    double X,
    double Y,
    double Z)
{
    public static Vector3D Zero { get; } = new(0.0, 0.0, 0.0);

    public double Length => Math.Sqrt(LengthSquared);

    public double LengthSquared =>
        (X * X) + (Y * Y) + (Z * Z);

    public Vector3D Normalize()
    {
        if (LengthSquared == 0.0)
        {
            throw new InvalidOperationException(
            "A zero-length vector cannot be normalized.");
        }

        var inverseLength = 1.0 / Length;

        return new(
            X * inverseLength,
            Y * inverseLength,
            Z * inverseLength
        );
    }

    public static double Dot(Vector3D left, Vector3D right)
    {
        return (left.X * right.X) + (left.Y * right.Y) + (left.Z * right.Z);
    }

    public static Vector3D Cross(Vector3D left, Vector3D right)
    {
        return new(
            (left.Y * right.Z) - (left.Z * right.Y),
            (left.Z * right.X) - (left.X * right.Z),
            (left.X * right.Y) - (left.Y * right.X)
        );
    }

    public static Vector3D operator +(Vector3D left, Vector3D right)
    {
        return new(
            left.X + right.X,
            left.Y + right.Y,
            left.Z + right.Z
        );
    }

    public static Vector3D operator -(Vector3D left, Vector3D right)
    {
        return new(
            left.X - right.X,
            left.Y - right.Y,
            left.Z - right.Z
        );
    }

    public static Vector3D operator -(Vector3D vector)
    {
        return new(
            -vector.X,
            -vector.Y,
            -vector.Z
        );
    }

    public static Vector3D operator *(Vector3D vector, double scalar)
    {
        return new(
            vector.X * scalar,
            vector.Y * scalar,
            vector.Z * scalar
        );
    }

    public static Vector3D operator *(double scalar, Vector3D vector)
    {
        return vector * scalar;
    }
}
