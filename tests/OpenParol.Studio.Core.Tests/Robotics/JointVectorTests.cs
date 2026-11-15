using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Core.Tests.Robotics;

public sealed class JointVectorTests
{
    [Fact]
    public void ConstructorStoresAllJointValues()
    {
        var joints = new JointVector(
            J1: 1.0,
            J2: 2.0,
            J3: 3.0,
            J4: 4.0,
            J5: 5.0,
            J6: 6.0);

        Assert.Equal(1.0, joints.J1);
        Assert.Equal(2.0, joints.J2);
        Assert.Equal(3.0, joints.J3);
        Assert.Equal(4.0, joints.J4);
        Assert.Equal(5.0, joints.J5);
        Assert.Equal(6.0, joints.J6);
    }

    [Theory]
    [InlineData(0, 1.0)]
    [InlineData(1, 2.0)]
    [InlineData(2, 3.0)]
    [InlineData(3, 4.0)]
    [InlineData(4, 5.0)]
    [InlineData(5, 6.0)]
    public void IndexerReturnsExpectedJointValue(
        int index,
        double expected)
    {
        var joints = new JointVector(1, 2, 3, 4, 5, 6);

        var actual = joints[index];

        Assert.Equal(expected, actual);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(6)]
    [InlineData(int.MinValue)]
    [InlineData(int.MaxValue)]
    public void IndexerThrowsForIndexOutsideValidRange(int index)
    {
        var joints = new JointVector(1, 2, 3, 4, 5, 6);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => _ = joints[index]);
    }

    [Fact]
    public void EqualVectorsAreEqual()
    {
        var first = new JointVector(1, 2, 3, 4, 5, 6);
        var second = new JointVector(1, 2, 3, 4, 5, 6);

        Assert.Equal(first, second);
    }

    [Fact]
    public void CountMatchesNumberOfRobotJoints()
    {
        Assert.Equal(6, JointVector.Count);
    }
}
