using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Inverse;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Inverse;

public sealed class DampedLeastSquaresSolverTests
{
    private const double PositionTolerance = 0.0001;

    private const double OrientationTolerance =
        Math.PI / 1800.0;

    [Fact]
    public void ConstructorThrowsForNullModel()
    {
        Assert.Throws<ArgumentNullException>(
            () => new DampedLeastSquaresSolver(null!)
        );
    }

    [Fact]
    public void SolveConvergesImmediatelyWhenInitialPoseMatchesTarget()
    {
        var model = CreateModel();
        var forwardSolver =
            new ForwardKinematicsSolver(model);

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var initialPositions = new JointVector(
            0.2,
            -0.3,
            0.4,
            -0.2,
            0.3,
            -0.4
        );

        var target =
            forwardSolver.Calculate(initialPositions)
                .ToolPose;

        var result = inverseSolver.Solve(
            target,
            initialPositions
        );

        Assert.True(result.Converged);

        Assert.Equal(
            InverseKinematicsStatus.Converged,
            result.Status
        );

        Assert.Equal(0, result.Iterations);
        Assert.Equal(initialPositions, result.JointPositions);

        Assert.True(
            result.PositionError <= PositionTolerance
        );

        Assert.True(
            result.OrientationError
                <= OrientationTolerance
        );
    }

    [Fact]
    public void SolveTreatsOppositeQuaternionSignAsSameOrientation()
    {
        var model = CreateModel();
        var forwardSolver =
            new ForwardKinematicsSolver(model);

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var initialPositions = new JointVector(
            0.2,
            -0.3,
            0.4,
            -0.2,
            0.3,
            -0.4
        );

        var sourcePose =
            forwardSolver.Calculate(initialPositions)
                .ToolPose;

        var target = new RobotPose(
            sourcePose.Position,
            new QuaternionD(
                -sourcePose.Orientation.X,
                -sourcePose.Orientation.Y,
                -sourcePose.Orientation.Z,
                -sourcePose.Orientation.W
            )
        );

        var result = inverseSolver.Solve(
            target,
            initialPositions
        );

        Assert.True(result.Converged);
        Assert.Equal(0, result.Iterations);
        Assert.Equal(initialPositions, result.JointPositions);
    }

    [Fact]
    public void SolveFindsNearbyParol6Configuration()
    {
        var model = CreateModel();

        var forwardSolver =
            new ForwardKinematicsSolver(model);

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var targetPositions = new JointVector(
            0.30,
            -0.40,
            0.65,
            -0.35,
            0.45,
            -0.55
        );

        var initialPositions = new JointVector(
            0.25,
            -0.35,
            0.60,
            -0.30,
            0.40,
            -0.50
        );

        var target =
            forwardSolver.Calculate(targetPositions)
                .ToolPose;

        var options = CreateTestOptions();

        var result = inverseSolver.Solve(
            target,
            initialPositions,
            options
        );

        Assert.True(
            result.Converged,
            $"Expected IK to converge, but received "
                + $"{result.Status}. Position error: "
                + $"{result.PositionError} m. Orientation error: "
                + $"{result.OrientationError} rad."
        );

        AssertResultPoseMatchesTarget(
            forwardSolver,
            result,
            target,
            options
        );
    }

    [Fact]
    public void SolveFindsPoseWithoutRequiringOriginalJointValues()
    {
        var model = CreateModel();

        var forwardSolver =
            new ForwardKinematicsSolver(model);

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var targetPositions = new JointVector(
            -0.35,
            0.25,
            0.50,
            0.40,
            -0.30,
            0.65
        );

        var initialPositions = new JointVector(
            -0.25,
            0.15,
            0.40,
            0.30,
            -0.20,
            0.55
        );

        var target =
            forwardSolver.Calculate(targetPositions)
                .ToolPose;

        var options = CreateTestOptions();

        var result = inverseSolver.Solve(
            target,
            initialPositions,
            options
        );

        Assert.True(
            result.Converged,
            $"Expected IK to converge, but received "
                + $"{result.Status}. Position error: "
                + $"{result.PositionError} m. Orientation error: "
                + $"{result.OrientationError} rad."
        );

        AssertResultPoseMatchesTarget(
            forwardSolver,
            result,
            target,
            options
        );
    }

    [Fact]
    public void SolveRespectsPositionAndOrientationTolerances()
    {
        var model = CreateModel();

        var forwardSolver =
            new ForwardKinematicsSolver(model);

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var targetPositions = new JointVector(
            0.40,
            -0.35,
            0.55,
            -0.45,
            0.50,
            -0.60
        );

        var initialPositions = new JointVector(
            0.30,
            -0.25,
            0.45,
            -0.35,
            0.40,
            -0.50
        );

        var target =
            forwardSolver.Calculate(targetPositions)
                .ToolPose;

        var options = CreateTestOptions();

        var result = inverseSolver.Solve(
            target,
            initialPositions,
            options
        );

        Assert.True(result.Converged);

        Assert.True(
            result.PositionError
                <= options.PositionTolerance,
            $"Position error {result.PositionError} m "
                + $"exceeded tolerance "
                + $"{options.PositionTolerance} m."
        );

        Assert.True(
            result.OrientationError
                <= options.OrientationTolerance,
            $"Orientation error {result.OrientationError} rad "
                + $"exceeded tolerance "
                + $"{options.OrientationTolerance} rad."
        );
    }

    [Fact]
    public void SolveReturnsFailureForClearlyUnreachableTarget()
    {
        var model = CreateModel();

        var inverseSolver =
            new DampedLeastSquaresSolver(model);

        var target = new RobotPose(
            new Vector3D(
                10.0,
                10.0,
                10.0
            ),
            QuaternionD.Identity
        );

        var options = CreateTestOptions() with
        {
            MaximumIterations = 50
        };

        var result = inverseSolver.Solve(
            target,
            ZeroJointVector(),
            options
        );

        Assert.False(result.Converged);

        Assert.True(
            result.Status
                is InverseKinematicsStatus.JointLimitReached
                or InverseKinematicsStatus.MaximumIterationsReached
        );

        Assert.Equal(
            options.MaximumIterations,
            result.Iterations
        );

        Assert.True(result.PositionError > 1.0);
    }

    [Fact]
    public void SolveThrowsForInitialPositionBelowJointLimit()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var initialPositions = new JointVector(
            -1.8,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        Assert.Throws<ArgumentOutOfRangeException>(
            () => solver.Solve(
                RobotPose.Identity,
                initialPositions
            )
        );
    }

    [Fact]
    public void SolveThrowsForInitialPositionAboveJointLimit()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var initialPositions = new JointVector(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            3.2
        );

        Assert.Throws<ArgumentOutOfRangeException>(
            () => solver.Solve(
                RobotPose.Identity,
                initialPositions
            )
        );
    }

    [Fact]
    public void SolveThrowsForNonFiniteInitialPosition()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var initialPositions = new JointVector(
            0.0,
            0.0,
            double.NaN,
            0.0,
            0.0,
            0.0
        );

        Assert.Throws<ArgumentException>(
            () => solver.Solve(
                RobotPose.Identity,
                initialPositions
            )
        );
    }

    [Fact]
    public void SolveThrowsForNonFiniteTargetPosition()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var target = new RobotPose(
            new Vector3D(
                double.NaN,
                0.0,
                0.0
            ),
            QuaternionD.Identity
        );

        Assert.Throws<ArgumentException>(
            () => solver.Solve(
                target,
                ZeroJointVector()
            )
        );
    }

    [Fact]
    public void SolveThrowsForZeroTargetOrientation()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var target = new RobotPose(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        Assert.Throws<ArgumentException>(
            () => solver.Solve(
                target,
                ZeroJointVector()
            )
        );
    }

    [Fact]
    public void SolveThrowsForNonFiniteTargetOrientation()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var target = new RobotPose(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                double.PositiveInfinity,
                1.0
            )
        );

        Assert.Throws<ArgumentException>(
            () => solver.Solve(
                target,
                ZeroJointVector()
            )
        );
    }

    [Fact]
    public void SolveValidatesProvidedOptions()
    {
        var solver =
            new DampedLeastSquaresSolver(
                CreateModel()
            );

        var options = new InverseKinematicsOptions
        {
            Damping = 0.0
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            () => solver.Solve(
                RobotPose.Identity,
                ZeroJointVector(),
                options
            )
        );
    }

    private static RobotKinematicModel CreateModel()
    {
        return Parol6KinematicModelFactory.Create(
            Parol6ToolDefinitions.PneumaticGripper
        );
    }

    private static InverseKinematicsOptions CreateTestOptions()
    {
        return new InverseKinematicsOptions
        {
            MaximumIterations = 200,

            PositionTolerance =
                PositionTolerance,

            OrientationTolerance =
                OrientationTolerance,

            Damping = 0.01,

            MaximumJointStep =
                5.0 * Math.PI / 180.0,

            PositionWeight = 1.0,

            OrientationWeight = 0.5
        };
    }

    private static JointVector ZeroJointVector()
    {
        return new JointVector(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );
    }

    private static void AssertResultPoseMatchesTarget(
        ForwardKinematicsSolver forwardSolver,
        InverseKinematicsResult result,
        RobotPose target,
        InverseKinematicsOptions options)
    {
        var actualPose =
            forwardSolver.Calculate(
                result.JointPositions
            ).ToolPose;

        var finalError =
            PoseError.Calculate(
                actualPose,
                target
            );

        Assert.True(
            finalError.PositionMagnitude
                <= options.PositionTolerance,
            $"Final position error "
                + $"{finalError.PositionMagnitude} m exceeded "
                + $"{options.PositionTolerance} m."
        );

        Assert.True(
            finalError.OrientationMagnitude
                <= options.OrientationTolerance,
            $"Final orientation error "
                + $"{finalError.OrientationMagnitude} rad exceeded "
                + $"{options.OrientationTolerance} rad."
        );
    }
}
