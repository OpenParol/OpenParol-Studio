using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Inverse;

namespace OpenParol.Studio.Kinematics.Tests.Inverse;

public sealed class InverseKinematicsResultTests
{
    [Fact]
    public void ConstructorStoresResultData()
    {
        var joints = new JointVector(
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0
        );

        var result = new InverseKinematicsResult(
            InverseKinematicsStatus.Converged,
            joints,
            12,
            0.00005,
            0.0001
        );

        Assert.Equal(
            InverseKinematicsStatus.Converged,
            result.Status
        );

        Assert.Equal(joints, result.JointPositions);
        Assert.Equal(12, result.Iterations);
        Assert.Equal(0.00005, result.PositionError);
        Assert.Equal(0.0001, result.OrientationError);
        Assert.True(result.Converged);
    }

    [Fact]
    public void ConvergedIsFalseForFailureStatus()
    {
        var result = new InverseKinematicsResult(
            InverseKinematicsStatus.MaximumIterationsReached,
            default,
            100,
            0.01,
            0.02
        );

        Assert.False(result.Converged);
    }

    [Fact]
    public void ConstructorThrowsForNegativeIterationCount()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new InverseKinematicsResult(
                InverseKinematicsStatus.Converged,
                default,
                -1,
                0.0,
                0.0
            )
        );
    }

    [Theory]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ConstructorThrowsForInvalidPositionError(
        double value)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new InverseKinematicsResult(
                InverseKinematicsStatus.Converged,
                default,
                0,
                value,
                0.0
            )
        );
    }

    [Theory]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ConstructorThrowsForInvalidOrientationError(
        double value)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new InverseKinematicsResult(
                InverseKinematicsStatus.Converged,
                default,
                0,
                0.0,
                value
            )
        );
    }
}
