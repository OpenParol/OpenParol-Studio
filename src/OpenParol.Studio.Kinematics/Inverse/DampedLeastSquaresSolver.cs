using MathNet.Numerics.LinearAlgebra;
using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Jacobians;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Inverse;

/// <summary>
/// Solves inverse kinematics using the damped least-squares method.
/// </summary>
public sealed class DampedLeastSquaresSolver
{
    private readonly RobotKinematicModel _model;

    private readonly ForwardKinematicsSolver _forwardKinematicsSolver;

    private readonly GeometricJacobianCalculator _jacobianCalculator;

    public DampedLeastSquaresSolver(
        RobotKinematicModel model)
    {
        ArgumentNullException.ThrowIfNull(model);

        _model = model;
        _forwardKinematicsSolver =
            new ForwardKinematicsSolver(model);

        _jacobianCalculator =
            new GeometricJacobianCalculator(model);
    }

    public InverseKinematicsResult Solve(
        RobotPose target,
        JointVector initialPositions,
        InverseKinematicsOptions? options = null)
    {
        options ??= new InverseKinematicsOptions();
        options.Validate();

        ValidateTarget(target);
        ValidateInitialPositions(initialPositions);

        var currentPositions =
            initialPositions;

        var reachedJointLimit = false;

        for (var iteration = 0;
             iteration < options.MaximumIterations;
             iteration++)
        {
            var forwardResult =
                _forwardKinematicsSolver.Calculate(
                    currentPositions
                );

            var error = PoseError.Calculate(
                forwardResult.ToolPose,
                target
            );

            if (HasConverged(error, options))
            {
                return CreateResult(
                    InverseKinematicsStatus.Converged,
                    currentPositions,
                    iteration,
                    error
                );
            }

            var jacobian =
                _jacobianCalculator.Calculate(
                    currentPositions
                );

            var jacobianMatrix =
                CreateJacobianMatrix(
                    jacobian,
                    options
                );

            var errorVector =
                CreateErrorVector(
                    error,
                    options
                );

            var jointStep =
                CalculateJointStep(
                    jacobianMatrix,
                    errorVector,
                    options.Damping
                );

            LimitJointStep(
                jointStep,
                options.MaximumJointStep
            );

            currentPositions = ApplyJointStep(
                currentPositions,
                jointStep,
                out var limitReachedThisIteration
            );

            reachedJointLimit |=
                limitReachedThisIteration;
        }

        var finalForwardResult =
            _forwardKinematicsSolver.Calculate(
                currentPositions
            );

        var finalError =
            PoseError.Calculate(
                finalForwardResult.ToolPose,
                target
            );

        var status = reachedJointLimit
            ? InverseKinematicsStatus.JointLimitReached
            : InverseKinematicsStatus.MaximumIterationsReached;

        return CreateResult(
            status,
            currentPositions,
            options.MaximumIterations,
            finalError
        );
    }

    private static void ValidateTarget(
        RobotPose target)
    {
        if (!IsFinite(target.Position))
        {
            throw new ArgumentException(
                "The target position must contain finite values.",
                nameof(target)
            );
        }

        if (!IsFinite(target.Orientation)
            || target.Orientation.LengthSquared == 0.0)
        {
            throw new ArgumentException(
                "The target orientation must be a finite, "
                    + "non-zero quaternion.",
                nameof(target)
            );
        }
    }

    private static bool IsFinite(
        Vector3D vector)
    {
        return double.IsFinite(vector.X)
            && double.IsFinite(vector.Y)
            && double.IsFinite(vector.Z);
    }

    private static bool IsFinite(
        QuaternionD quaternion)
    {
        return double.IsFinite(quaternion.X)
            && double.IsFinite(quaternion.Y)
            && double.IsFinite(quaternion.Z)
            && double.IsFinite(quaternion.W);
    }

    private void ValidateInitialPositions(
        JointVector positions)
    {
        for (var index = 0;
             index < JointVector.Count;
             index++)
        {
            var value = positions[index];
            var joint = _model.Joints[index];

            if (!double.IsFinite(value))
            {
                throw new ArgumentException(
                    $"The initial position of joint "
                        + $"'{joint.Name}' must be finite.",
                    nameof(positions)
                );
            }

            if (!joint.PositionLimits.Contains(value))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(positions),
                    value,
                    $"The initial position of joint '{joint.Name}' "
                        + "is outside its limits."
                );
            }
        }
    }

    private static Matrix<double> CreateJacobianMatrix(
        RobotJacobian jacobian,
        InverseKinematicsOptions options)
    {
        var matrix = Matrix<double>.Build.Dense(
            rows: 6,
            columns: JointVector.Count
        );

        for (var column = 0;
             column < JointVector.Count;
             column++)
        {
            var value = jacobian[column];

            matrix[0, column] =
                value.Linear.X * options.PositionWeight;

            matrix[1, column] =
                value.Linear.Y * options.PositionWeight;

            matrix[2, column] =
                value.Linear.Z * options.PositionWeight;

            matrix[3, column] =
                value.Angular.X * options.OrientationWeight;

            matrix[4, column] =
                value.Angular.Y * options.OrientationWeight;

            matrix[5, column] =
                value.Angular.Z * options.OrientationWeight;
        }

        return matrix;
    }

    private static Vector<double> CreateErrorVector(
        PoseError error,
        InverseKinematicsOptions options)
    {
        return Vector<double>.Build.Dense(
        [
            error.Position.X * options.PositionWeight,
            error.Position.Y * options.PositionWeight,
            error.Position.Z * options.PositionWeight,

            error.Orientation.X * options.OrientationWeight,
            error.Orientation.Y * options.OrientationWeight,
            error.Orientation.Z * options.OrientationWeight
        ]);
    }

    private static Vector<double> CalculateJointStep(
        Matrix<double> jacobian,
        Vector<double> error,
        double damping)
    {
        var transpose =
            jacobian.Transpose();

        var systemMatrix =
            transpose * jacobian;

        var dampingSquared =
            damping * damping;

        for (var index = 0;
            index < JointVector.Count;
            index++)
        {
            systemMatrix[index, index] +=
                dampingSquared;
        }

        var rightHandSide =
            transpose * error;

        return systemMatrix.Solve(
            rightHandSide
        );
    }

    private static void LimitJointStep(
        Vector<double> jointStep,
        double maximumJointStep)
    {
        var largestStep = jointStep
            .Max(Math.Abs);

        if (largestStep <= maximumJointStep)
        {
            return;
        }

        var scale =
            maximumJointStep / largestStep;

        jointStep.Multiply(
            scale,
            jointStep
        );
    }

    private JointVector ApplyJointStep(
        JointVector positions,
        Vector<double> jointStep,
        out bool reachedLimit)
    {
        reachedLimit = false;

        var values = new double[JointVector.Count];

        for (var index = 0;
            index < JointVector.Count;
            index++)
        {
            var limits =
                _model.Joints[index].PositionLimits;

            var requested =
                positions[index] + jointStep[index];

            var limited =
                limits.Clamp(requested);

            if (limited != requested)
            {
                reachedLimit = true;
            }

            values[index] = limited;
        }

        return new JointVector(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );
    }

    private static bool HasConverged(
        PoseError error,
        InverseKinematicsOptions options)
    {
        return error.PositionMagnitude
                <= options.PositionTolerance
            && error.OrientationMagnitude
                <= options.OrientationTolerance;
    }

    private static InverseKinematicsResult CreateResult(
        InverseKinematicsStatus status,
        JointVector positions,
        int iterations,
        PoseError error)
    {
        return new InverseKinematicsResult(
            status,
            positions,
            iterations,
            error.PositionMagnitude,
            error.OrientationMagnitude
        );
    }
}
