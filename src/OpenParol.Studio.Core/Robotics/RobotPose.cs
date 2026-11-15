using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Core.Robotics;

/// <summary>
/// Represents the position and orientation of the robot tool center point.
/// </summary>
/// <remarks>
/// Position values are expressed in meters relative to the robot base frame.
/// Orientation is represented by a quaternion relative to the robot base frame.
/// </remarks>
public readonly record struct RobotPose(
    Vector3D Position,
    QuaternionD Orientation)
{
    public static RobotPose Identity { get; } = new(
        Vector3D.Zero,
        QuaternionD.Identity
    );
}
