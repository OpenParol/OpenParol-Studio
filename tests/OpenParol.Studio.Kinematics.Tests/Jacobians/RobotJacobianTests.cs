using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Jacobians;

namespace OpenParol.Studio.Kinematics.Tests.Jacobians;

public sealed class RobotJacobianTests
{
    [Fact]
    public void ConstructorStoresSixColumns()
    {
        var columns = CreateColumns(
            JointVector.Count
        );

        var jacobian =
            new RobotJacobian(columns);

        Assert.Equal(
            JointVector.Count,
            jacobian.Columns.Count
        );

        for (var i = 0; i < JointVector.Count; i++)
        {
            Assert.Equal(
                columns[i],
                jacobian[i]
            );
        }
    }

    [Fact]
    public void ConstructorCopiesColumnCollection()
    {
        var columns = CreateColumns(
            JointVector.Count
        );

        var jacobian =
            new RobotJacobian(columns);

        columns[0] = new JacobianColumn(
            new Vector3D(100.0, 0.0, 0.0),
            Vector3D.Zero
        );

        Assert.NotEqual(
            columns[0],
            jacobian[0]
        );
    }

    [Fact]
    public void ConstructorThrowsForNullColumns()
    {
        Assert.Throws<ArgumentNullException>(
            () => new RobotJacobian(null!)
        );
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(5)]
    [InlineData(7)]
    public void ConstructorThrowsForInvalidColumnCount(
        int count)
    {
        Assert.Throws<ArgumentException>(
            () => new RobotJacobian(
                CreateColumns(count)
            )
        );
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(6)]
    public void IndexerThrowsForInvalidIndex(
        int index)
    {
        var jacobian = new RobotJacobian(
            CreateColumns(JointVector.Count)
        );

        Assert.Throws<ArgumentOutOfRangeException>(
            () => _ = jacobian[index]
        );
    }

    private static JacobianColumn[] CreateColumns(
        int count)
    {
        return Enumerable
            .Range(0, count)
            .Select(index =>
                new JacobianColumn(
                    new Vector3D(
                        index,
                        index + 1.0,
                        index + 2.0
                    ),
                    new Vector3D(
                        0.0,
                        0.0,
                        1.0
                    )
                )
            )
            .ToArray();
    }
}
