using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Forward;

/// <summary>
/// Calculates forward kinematics for a six-axis serial robot.
/// </summary>
public sealed class ForwardKinematicsSolver
{
    private readonly RobotKinematicModel _model;

    public ForwardKinematicsSolver(
        RobotKinematicModel model)
    {
        ArgumentNullException.ThrowIfNull(model);

        _model = model;
    }

    public ForwardKinematicsResult Calculate(
        JointVector jointPositions)
    {
        var jointFrames =
            new RigidTransform[JointVector.Count];

        var linkFrames =
            new RigidTransform[JointVector.Count];

        var current = _model.BaseTransform;

        for (var i = 0; i < _model.Joints.Count; i++)
        {
            var joint = _model.Joints[i];
            var position = jointPositions[i];

            if (!joint.PositionLimits.Contains(position))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(jointPositions),
                    position,
                    $"The position of joint '{joint.Name}' must be "
                        + $"between {joint.PositionLimits.Minimum} and "
                        + $"{joint.PositionLimits.Maximum} radians."
                );
            }

            current *= joint.ParentToJoint;

            jointFrames[i] = current;

            var angle =
                position + joint.ZeroOffset;

            var jointRotation =
                RigidTransform.FromRotation(
                    QuaternionD.FromAxisAngle(
                        joint.Axis,
                        angle
                    )
                );

            current *= jointRotation;

            linkFrames[i] = current;
        }

        var worldToFlange = current * _model.LastLinkToFlange;

        var worldToTool = worldToFlange * _model.FlangeToTool;

        return new ForwardKinematicsResult(
            jointFrames,
            linkFrames,
            worldToFlange,
            worldToTool
        );
    }
}
