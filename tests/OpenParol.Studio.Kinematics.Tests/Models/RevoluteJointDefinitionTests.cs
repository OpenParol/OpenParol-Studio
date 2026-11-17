using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Models;

public sealed class RevoluteJointDefinitionTests
{
    private const int Precision = 12;

    [Fact]
    public void ConstructorStoresJointDefinition()
    {
        var transform = RigidTransform.FromTranslation(
            new Vector3D(0.0, 0.0, 0.25)
        );

        var limits = new JointPositionLimits(
            -Math.PI,
            Math.PI
        );

        var joint = new RevoluteJointDefinition(
            "J1",
            new Vector3D(0.0, 0.0, 1.0),
            transform,
            limits,
            0.25
        );

        Assert.Equal("J1", joint.Name);
        Assert.Equal(transform, joint.ParentToJoint);
        Assert.Equal(limits, joint.PositionLimits);
        Assert.Equal(0.25, joint.ZeroOffset);
    }

    [Fact]
    public void ConstructorNormalizesJointAxis()
    {
        var joint = CreateJoint(
            new Vector3D(0.0, 0.0, 10.0)
        );

        Assert.Equal(0.0, joint.Axis.X, Precision);
        Assert.Equal(0.0, joint.Axis.Y, Precision);
        Assert.Equal(1.0, joint.Axis.Z, Precision);
        Assert.Equal(1.0, joint.Axis.Length, Precision);
    }

    [Theory]
    [InlineData("")]
    [InlineData(" ")]
    [InlineData("   ")]
    public void ConstructorThrowsForEmptyName(
        string name)
    {
        Assert.Throws<ArgumentException>(
            () => new RevoluteJointDefinition(
                name,
                new Vector3D(0.0, 0.0, 1.0),
                RigidTransform.Identity,
                new JointPositionLimits(-1.0, 1.0)
            )
        );
    }

    [Fact]
    public void ConstructorThrowsForNullName()
    {
        Assert.Throws<ArgumentException>(
            () => new RevoluteJointDefinition(
                null!,
                new Vector3D(0.0, 0.0, 1.0),
                RigidTransform.Identity,
                new JointPositionLimits(-1.0, 1.0)
            )
        );
    }

    [Fact]
    public void ConstructorThrowsForZeroAxis()
    {
        Assert.Throws<ArgumentException>(
            () => CreateJoint(Vector3D.Zero)
        );
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void ConstructorThrowsForInvalidZeroOffset(
        double zeroOffset)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new RevoluteJointDefinition(
                "J1",
                new Vector3D(0.0, 0.0, 1.0),
                RigidTransform.Identity,
                new JointPositionLimits(-1.0, 1.0),
                zeroOffset
            )
        );
    }

    private static RevoluteJointDefinition CreateJoint(
        Vector3D axis)
    {
        return new RevoluteJointDefinition(
            "J1",
            axis,
            RigidTransform.Identity,
            new JointPositionLimits(-Math.PI, Math.PI)
        );
    }
}
