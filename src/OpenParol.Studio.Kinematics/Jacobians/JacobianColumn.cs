using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Kinematics.Jacobians;

/// <summary>
/// Represents one column of a geometric robot Jacobian.
/// </summary>
public readonly record struct JacobianColumn(
    Vector3D Linear,
    Vector3D Angular);
