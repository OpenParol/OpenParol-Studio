using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Jacobians;

/// <summary>
/// Calculates the geometric Jacobian of a six-axis serial robot.
/// </summary>
public sealed class GeometricJacobianCalculator
{
    private readonly RobotKinematicModel _model;

    private readonly ForwardKinematicsSolver _forwardKinematicsSolver;

    public GeometricJacobianCalculator(
        RobotKinematicModel model)
    {
        ArgumentNullException.ThrowIfNull(model);

        _model = model;
        _forwardKinematicsSolver =
            new ForwardKinematicsSolver(model);
    }

    public RobotJacobian Calculate(
        JointVector jointPositions)
    {
        var forwardResult =
            _forwardKinematicsSolver.Calculate(
                jointPositions
            );

        var toolOrigin =
            forwardResult.WorldToTool.Translation;

        var columns =
            new JacobianColumn[JointVector.Count];

        for (var i = 0; i < JointVector.Count; i++)
        {
            var joint = _model.Joints[i];

            var jointFrame =
                forwardResult.WorldToJointFrames[i];

            var jointOrigin =
                jointFrame.Translation;

            var jointAxisInWorld =
                jointFrame.TransformDirection(
                    joint.Axis
                );

            var jointToTool =
                toolOrigin - jointOrigin;

            var linear =
                Vector3D.Cross(
                    jointAxisInWorld,
                    jointToTool
                );

            var angular =
                jointAxisInWorld;

            columns[i] = new JacobianColumn(
                linear,
                angular
            );
        }

        return new RobotJacobian(columns);
    }
}
