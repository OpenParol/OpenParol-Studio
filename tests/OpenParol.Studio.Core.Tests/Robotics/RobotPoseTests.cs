using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Core.Tests.Robotics;

public sealed class RobotPoseTests
{
    [Fact]
    public void ConstructorStoresPositionAndOrientation()
    {
        var position = new Vector3D(
            0.35,
            0.10,
            0.42
        );

        var orientation = QuaternionD.FromAxisAngle(
            new Vector3D(0.0, 0.0, 1.0),
            Math.PI / 2.0
        );

        var pose = new RobotPose(
            position,
            orientation
        );

        Assert.Equal(position, pose.Position);
        Assert.Equal(orientation, pose.Orientation);
    }

    [Fact]
    public void IdentityHasZeroPosition()
    {
        Assert.Equal(
            Vector3D.Zero,
            RobotPose.Identity.Position
        );
    }

    [Fact]
    public void IdentityHasIdentityOrientation()
    {
        Assert.Equal(
            QuaternionD.Identity,
            RobotPose.Identity.Orientation
        );
    }

    [Fact]
    public void PosesWithEqualValuesAreEqual()
    {
        var first = new RobotPose(
            new Vector3D(0.35, 0.10, 0.42),
            QuaternionD.Identity
        );

        var second = new RobotPose(
            new Vector3D(0.35, 0.10, 0.42),
            QuaternionD.Identity
        );

        Assert.Equal(first, second);
    }

    [Fact]
    public void PosesWithDifferentPositionsAreNotEqual()
    {
        var first = new RobotPose(
            new Vector3D(0.35, 0.10, 0.42),
            QuaternionD.Identity
        );

        var second = new RobotPose(
            new Vector3D(0.36, 0.10, 0.42),
            QuaternionD.Identity
        );

        Assert.NotEqual(first, second);
    }

    [Fact]
    public void PosesWithDifferentOrientationsAreNotEqual()
    {
        var first = new RobotPose(
            Vector3D.Zero,
            QuaternionD.Identity
        );

        var second = new RobotPose(
            Vector3D.Zero,
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            )
        );

        Assert.NotEqual(first, second);
    }
}
