using System.Collections.ObjectModel;
using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Forward;

/// <summary>
/// Contains the result of evaluating the complete forward-kinematics chain.
/// </summary>
public sealed class ForwardKinematicsResult
{
    private readonly ReadOnlyCollection<RigidTransform> _worldToJointFrames;

    private readonly ReadOnlyCollection<RigidTransform> _worldToLinkFrames;

    /// <summary>
    /// Gets the fixed joint frames expressed in world coordinates,
    /// before applying each joint rotation.
    /// </summary>
    public IReadOnlyList<RigidTransform> WorldToJointFrames =>
        _worldToJointFrames;

    /// <summary>
    /// Gets the moving link frames expressed in world coordinates,
    /// after applying each joint rotation.
    /// </summary>
    public IReadOnlyList<RigidTransform> WorldToLinkFrames =>
        _worldToLinkFrames;

    public RigidTransform WorldToFlange { get; }

    public RigidTransform WorldToTool { get; }

    public RobotPose ToolPose { get; }

    public ForwardKinematicsResult(
        IEnumerable<RigidTransform> worldToJointFrames,
        IEnumerable<RigidTransform> worldToLinkFrames,
        RigidTransform worldToFlange,
        RigidTransform worldToTool)
    {
        ArgumentNullException.ThrowIfNull(worldToJointFrames);
        ArgumentNullException.ThrowIfNull(worldToLinkFrames);

        var jointFrames = worldToJointFrames.ToArray();
        var linkFrames = worldToLinkFrames.ToArray();

        if (jointFrames.Length != JointVector.Count)
        {
            throw new ArgumentException(
                $"Forward kinematics must provide exactly "
                + $"{JointVector.Count} joint frames.",
                nameof(worldToJointFrames)
            );
        }

        if (linkFrames.Length != JointVector.Count)
        {
            throw new ArgumentException(
                $"Forward kinematics must provide exactly "
                + $"{JointVector.Count} link frames.",
                nameof(worldToLinkFrames)
            );
        }

        _worldToJointFrames = Array.AsReadOnly(jointFrames);
        _worldToLinkFrames = Array.AsReadOnly(linkFrames);

        WorldToFlange = worldToFlange;
        WorldToTool = worldToTool;

        ToolPose = new RobotPose(
            worldToTool.Translation,
            worldToTool.Rotation
        );
    }
}
