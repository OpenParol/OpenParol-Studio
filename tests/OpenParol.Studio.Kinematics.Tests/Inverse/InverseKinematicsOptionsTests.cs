using OpenParol.Studio.Kinematics.Inverse;

namespace OpenParol.Studio.Kinematics.Tests.Inverse;

public sealed class InverseKinematicsOptionsTests
{
    [Fact]
    public void DefaultOptionsAreValid()
    {
        var options = new InverseKinematicsOptions();

        options.Validate();
    }

    [Fact]
    public void ValidateThrowsForNonPositiveMaximumIterations()
    {
        var options = new InverseKinematicsOptions
        {
            MaximumIterations = 0
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidPositionTolerance(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            PositionTolerance = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidOrientationTolerance(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            OrientationTolerance = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidDamping(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            Damping = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidMaximumJointStep(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            MaximumJointStep = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidPositionWeight(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            PositionWeight = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ValidateThrowsForInvalidOrientationWeight(
        double value)
    {
        var options = new InverseKinematicsOptions
        {
            OrientationWeight = value
        };

        Assert.Throws<ArgumentOutOfRangeException>(
            options.Validate
        );
    }
}
