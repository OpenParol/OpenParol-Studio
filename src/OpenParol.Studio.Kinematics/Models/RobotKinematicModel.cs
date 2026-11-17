using System.Collections.ObjectModel;
using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Models;

/// <summary>
/// Defines the geometry of a six-axis serial robot.
/// </summary>
public sealed class RobotKinematicModel
{
    private readonly ReadOnlyCollection<RevoluteJointDefinition> _joints;

    public IReadOnlyList<RevoluteJointDefinition> Joints => _joints;

    public RigidTransform BaseTransform { get; }

    public RigidTransform FlangeToTool { get; }

    public RobotKinematicModel(
        IEnumerable<RevoluteJointDefinition> joints,
        RigidTransform baseTransform,
        RigidTransform flangeToTool)
    {
        ArgumentNullException.ThrowIfNull(joints);

        var jointArray = joints.ToArray();

        if (jointArray.Length != JointVector.Count)
        {
            throw new ArgumentException(
                $"The robot model must contain exactly "
                + $"{JointVector.Count} joints.",
                nameof(joints)
            );
        }

        if (jointArray.Any(joint => joint is null))
        {
            throw new ArgumentException(
                "The robot model cannot contain null joints.",
                nameof(joints)
            );
        }

        var duplicateName = jointArray
            .GroupBy(
                joint => joint.Name,
                StringComparer.OrdinalIgnoreCase
            )
            .FirstOrDefault(group => group.Count() > 1)
            ?.Key;

        if (duplicateName is not null)
        {
            throw new ArgumentException(
                $"The joint name '{duplicateName}' is duplicated.",
                nameof(joints)
            );
        }

        _joints = Array.AsReadOnly(jointArray);
        BaseTransform = baseTransform;
        FlangeToTool = flangeToTool;
    }
}
