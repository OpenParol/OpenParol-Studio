using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Models;

/// <summary>
/// Defines a revolute joint in a serial kinematic chain.
/// </summary>
public sealed class RevoluteJointDefinition
{
    public string Name { get; }

    public Vector3D Axis { get; }

    public RigidTransform ParentToJoint { get; }

    public JointPositionLimits PositionLimits { get; }

    public double ZeroOffset { get; }

    public RevoluteJointDefinition(
        string name,
        Vector3D axis,
        RigidTransform parentToJoint,
        JointPositionLimits positionLimits,
        double zeroOffset = 0.0)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            throw new ArgumentException(
                "The joint name cannot be empty.",
                nameof(name)
            );
        }

        if (axis.LengthSquared == 0.0)
        {
            throw new ArgumentException(
                "The joint axis cannot be a zero-length vector.",
                nameof(axis)
            );
        }

        if (!double.IsFinite(zeroOffset))
        {
            throw new ArgumentOutOfRangeException(
                nameof(zeroOffset),
                zeroOffset,
                "The joint zero offset must be finite."
            );
        }

        Name = name;
        Axis = axis.Normalize();
        ParentToJoint = parentToJoint;
        PositionLimits = positionLimits;
        ZeroOffset = zeroOffset;
    }
}
