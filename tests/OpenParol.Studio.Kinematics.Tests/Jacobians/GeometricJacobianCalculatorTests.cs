using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Jacobians;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Jacobians;

public sealed class GeometricJacobianCalculatorTests
{
    private const double PositionStep = 1e-6;

    private const double LinearTolerance = 1e-8;

    private const double AngularTolerance = 1e-8;

    [Fact]
    public void ConstructorThrowsForNullModel()
    {
        Assert.Throws<ArgumentNullException>(
            () => new GeometricJacobianCalculator(null!)
        );
    }

    [Fact]
    public void SingleZAxisJointWithToolOnXAxisProducesYVelocity()
    {
        var joints = CreateIdentityJoints();

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity,
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 0.0, 0.0)
            )
        );

        var calculator =
            new GeometricJacobianCalculator(model);

        var result = calculator.Calculate(
            ZeroJointVector()
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result[0].Linear,
            LinearTolerance
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 0.0, 1.0),
            result[0].Angular,
            AngularTolerance
        );
    }

    [Fact]
    public void JointDoesNotProduceLinearVelocityWhenToolLiesOnItsAxis()
    {
        var joints = CreateIdentityJoints();

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity,
            RigidTransform.FromTranslation(
                new Vector3D(0.0, 0.0, 1.0)
            )
        );

        var calculator =
            new GeometricJacobianCalculator(model);

        var result = calculator.Calculate(
            ZeroJointVector()
        );

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result[0].Linear,
            LinearTolerance
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 0.0, 1.0),
            result[0].Angular,
            AngularTolerance
        );
    }

    [Fact]
    public void PreviousJointRotationChangesLaterJointAxisInWorld()
    {
        var joints = CreateIdentityJoints();

        joints[1] = CreateJoint(
            name: "J2",
            axis: new Vector3D(1.0, 0.0, 0.0)
        );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity,
            RigidTransform.FromTranslation(
                new Vector3D(0.0, 0.0, 1.0)
            )
        );

        var calculator =
            new GeometricJacobianCalculator(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = calculator.Calculate(positions);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result[1].Angular,
            AngularTolerance
        );
    }

    [Fact]
    public void Parol6LinearJacobianMatchesNumericalDerivative()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var calculator =
            new GeometricJacobianCalculator(model);

        var solver =
            new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            0.35,
            -0.40,
            0.70,
            -0.55,
            0.60,
            -0.80
        );

        var analytical =
            calculator.Calculate(positions);

        for (var jointIndex = 0;
             jointIndex < JointVector.Count;
             jointIndex++)
        {
            var numerical =
                CalculateNumericalLinearColumn(
                    solver,
                    positions,
                    jointIndex
                );

            AssertVectorApproximatelyEqual(
                numerical,
                analytical[jointIndex].Linear,
                LinearTolerance
            );
        }
    }

    [Fact]
    public void Parol6AngularJacobianMatchesNumericalDerivative()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var calculator =
            new GeometricJacobianCalculator(model);

        var solver =
            new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            0.35,
            -0.40,
            0.70,
            -0.55,
            0.60,
            -0.80
        );

        var analytical =
            calculator.Calculate(positions);

        for (var jointIndex = 0;
             jointIndex < JointVector.Count;
             jointIndex++)
        {
            var numerical =
                CalculateNumericalAngularColumn(
                    solver,
                    positions,
                    jointIndex
                );

            AssertVectorApproximatelyEqual(
                numerical,
                analytical[jointIndex].Angular,
                AngularTolerance
            );
        }
    }

    [Fact]
    public void Parol6JacobianMatchesNumericalDerivativeNearZeroConfiguration()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var calculator =
            new GeometricJacobianCalculator(model);

        var solver =
            new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            0.1,
            -0.1,
            0.1,
            -0.1,
            0.1,
            -0.1
        );

        var analytical =
            calculator.Calculate(positions);

        for (var jointIndex = 0;
             jointIndex < JointVector.Count;
             jointIndex++)
        {
            var numericalLinear =
                CalculateNumericalLinearColumn(
                    solver,
                    positions,
                    jointIndex
                );

            var numericalAngular =
                CalculateNumericalAngularColumn(
                    solver,
                    positions,
                    jointIndex
                );

            AssertVectorApproximatelyEqual(
                numericalLinear,
                analytical[jointIndex].Linear,
                LinearTolerance
            );

            AssertVectorApproximatelyEqual(
                numericalAngular,
                analytical[jointIndex].Angular,
                AngularTolerance
            );
        }
    }

    private static Vector3D CalculateNumericalLinearColumn(
        ForwardKinematicsSolver solver,
        JointVector positions,
        int jointIndex)
    {
        var lowerPositions = AddToJoint(
            positions,
            jointIndex,
            -PositionStep
        );

        var upperPositions = AddToJoint(
            positions,
            jointIndex,
            PositionStep
        );

        var lowerToolPosition =
            solver.Calculate(lowerPositions)
                .WorldToTool
                .Translation;

        var upperToolPosition =
            solver.Calculate(upperPositions)
                .WorldToTool
                .Translation;

        var difference =
            upperToolPosition - lowerToolPosition;

        return difference * (
            1.0 / (2.0 * PositionStep)
        );
    }

    private static Vector3D CalculateNumericalAngularColumn(
        ForwardKinematicsSolver solver,
        JointVector positions,
        int jointIndex)
    {
        var lowerPositions = AddToJoint(
            positions,
            jointIndex,
            -PositionStep
        );

        var upperPositions = AddToJoint(
            positions,
            jointIndex,
            PositionStep
        );

        var lowerRotation =
            solver.Calculate(lowerPositions)
                .WorldToTool
                .Rotation
                .Normalize();

        var upperRotation =
            solver.Calculate(upperPositions)
                .WorldToTool
                .Rotation
                .Normalize();

        var relativeRotation =
            (
                upperRotation
                * lowerRotation.Conjugate()
            ).Normalize();

        if (relativeRotation.W < 0.0)
        {
            relativeRotation = new QuaternionD(
                -relativeRotation.X,
                -relativeRotation.Y,
                -relativeRotation.Z,
                -relativeRotation.W
            );
        }

        var vectorPart = new Vector3D(
            relativeRotation.X,
            relativeRotation.Y,
            relativeRotation.Z
        );

        if (vectorPart.LengthSquared == 0.0)
        {
            return Vector3D.Zero;
        }

        var angle = 2.0 * Math.Atan2(
            vectorPart.Length,
            relativeRotation.W
        );

        var axis = vectorPart.Normalize();

        return axis * (
            angle / (2.0 * PositionStep)
        );
    }

    private static JointVector AddToJoint(
        JointVector positions,
        int jointIndex,
        double value)
    {
        return jointIndex switch
        {
            0 => positions with
            {
                J1 = positions.J1 + value
            },

            1 => positions with
            {
                J2 = positions.J2 + value
            },

            2 => positions with
            {
                J3 = positions.J3 + value
            },

            3 => positions with
            {
                J4 = positions.J4 + value
            },

            4 => positions with
            {
                J5 = positions.J5 + value
            },

            5 => positions with
            {
                J6 = positions.J6 + value
            },

            _ => throw new ArgumentOutOfRangeException(
                nameof(jointIndex),
                jointIndex,
                $"Joint index must be between 0 and "
                    + $"{JointVector.Count - 1}."
            )
        };
    }

    private static RevoluteJointDefinition[] CreateIdentityJoints()
    {
        return
        [
            CreateJoint("J1"),
            CreateJoint("J2"),
            CreateJoint("J3"),
            CreateJoint("J4"),
            CreateJoint("J5"),
            CreateJoint("J6")
        ];
    }

    private static RevoluteJointDefinition CreateJoint(
        string name,
        Vector3D? axis = null)
    {
        return new RevoluteJointDefinition(
            name,
            axis ?? new Vector3D(0.0, 0.0, 1.0),
            RigidTransform.Identity,
            new JointPositionLimits(
                -Math.PI,
                Math.PI
            )
        );
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

    private static void AssertVectorApproximatelyEqual(
        Vector3D expected,
        Vector3D actual,
        double tolerance)
    {
        Assert.True(
            Math.Abs(expected.X - actual.X) <= tolerance,
            $"Expected X = {expected.X}, "
                + $"but received X = {actual.X}."
        );

        Assert.True(
            Math.Abs(expected.Y - actual.Y) <= tolerance,
            $"Expected Y = {expected.Y}, "
                + $"but received Y = {actual.Y}."
        );

        Assert.True(
            Math.Abs(expected.Z - actual.Z) <= tolerance,
            $"Expected Z = {expected.Z}, "
                + $"but received Z = {actual.Z}."
        );
    }
}
