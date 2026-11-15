namespace OpenParol.Studio.Core.Geometry;

/// <summary>
/// Represents a rigid-body transformation composed of a rotation
/// followed by a translation.
/// </summary>
public readonly record struct RigidTransform(
    Vector3D Translation,
    QuaternionD Rotation)
{
    public static RigidTransform Identity { get; } = new(
        Vector3D.Zero,
        QuaternionD.Identity
    );

    public Vector3D TransformPoint(Vector3D point)
    {
        return Rotation.Rotate(point) + Translation;
    }

    public Vector3D TransformDirection(Vector3D direction)
    {
        return Rotation.Rotate(direction);
    }

    public RigidTransform Inverse()
    {
        var normalizedRotation = Rotation.Normalize();
        var inverseRotation = normalizedRotation.Conjugate();
        var inverseTranslation = inverseRotation.Rotate(-Translation);

        return new(
            inverseTranslation,
            inverseRotation
        );
    }

    public static RigidTransform FromTranslation(Vector3D translation)
    {
        return new(
            translation,
            QuaternionD.Identity
        );
    }

    public static RigidTransform FromRotation(QuaternionD rotation)
    {
        return new(
            Vector3D.Zero,
            rotation.Normalize()
        );
    }

    public static RigidTransform operator *(
        RigidTransform left,
        RigidTransform right)
    {
        var rotation =
            (left.Rotation * right.Rotation).Normalize();

        var translation =
            left.Rotation.Rotate(right.Translation)
            + left.Translation;

        return new(
            translation,
            rotation
        );
    }
}
