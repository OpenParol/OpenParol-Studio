using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Core.Tests.Robotics;

public sealed class JointPositionLimitsTests
{
    [Fact]
    public void ConstructorStoresMinimumAndMaximum()
    {
        var limits = new JointPositionLimits(
            -Math.PI,
            Math.PI
        );

        Assert.Equal(-Math.PI, limits.Minimum);
        Assert.Equal(Math.PI, limits.Maximum);
    }

    [Fact]
    public void RangeReturnsDifferenceBetweenMaximumAndMinimum()
    {
        var limits = new JointPositionLimits(
            -2.0,
            3.0
        );

        Assert.Equal(5.0, limits.Range);
    }

    [Fact]
    public void ConstructorAllowsEqualLimits()
    {
        var limits = new JointPositionLimits(
            1.5,
            1.5
        );

        Assert.Equal(1.5, limits.Minimum);
        Assert.Equal(1.5, limits.Maximum);
        Assert.Equal(0.0, limits.Range);
    }

    [Fact]
    public void ConstructorThrowsWhenMinimumIsGreaterThanMaximum()
    {
        Assert.Throws<ArgumentException>(
            () => new JointPositionLimits(
                2.0,
                1.0
            )
        );
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ConstructorThrowsWhenMinimumIsNotFinite(
        double minimum)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new JointPositionLimits(
                minimum,
                1.0
            )
        );
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ConstructorThrowsWhenMaximumIsNotFinite(
        double maximum)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new JointPositionLimits(
                -1.0,
                maximum
            )
        );
    }

    [Theory]
    [InlineData(-1.0)]
    [InlineData(0.0)]
    [InlineData(1.0)]
    public void ContainsReturnsTrueForPositionInsideLimits(
        double position)
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        Assert.True(limits.Contains(position));
    }

    [Theory]
    [InlineData(-1.0001)]
    [InlineData(1.0001)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ContainsReturnsFalseForPositionOutsideLimits(
        double position)
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        Assert.False(limits.Contains(position));
    }

    [Fact]
    public void ClampReturnsPositionInsideLimitsUnchanged()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        var result = limits.Clamp(0.25);

        Assert.Equal(0.25, result);
    }

    [Fact]
    public void ClampReturnsMinimumForPositionBelowMinimum()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        var result = limits.Clamp(-2.0);

        Assert.Equal(-1.0, result);
    }

    [Fact]
    public void ClampReturnsMaximumForPositionAboveMaximum()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        var result = limits.Clamp(2.0);

        Assert.Equal(1.0, result);
    }

    [Fact]
    public void ClampReturnsMinimumForNegativeInfinity()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        var result =
            limits.Clamp(double.NegativeInfinity);

        Assert.Equal(-1.0, result);
    }

    [Fact]
    public void ClampReturnsMaximumForPositiveInfinity()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        var result =
            limits.Clamp(double.PositiveInfinity);

        Assert.Equal(1.0, result);
    }

    [Fact]
    public void ClampThrowsForNaN()
    {
        var limits = new JointPositionLimits(
            -1.0,
            1.0
        );

        Assert.Throws<ArgumentOutOfRangeException>(
            () => limits.Clamp(double.NaN)
        );
    }
}
