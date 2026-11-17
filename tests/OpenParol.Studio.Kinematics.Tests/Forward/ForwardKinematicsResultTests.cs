using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;

namespace OpenParol.Studio.Kinematics.Tests.Forward;

public sealed class ForwardKinematicsResultTests
{
    [Fact]
    public void ConstructorStoresForwardKinematicsResult()
    {
        var jointFrames = CreateTransforms(
            translationOffset: 0.0
        );

        var linkFrames = CreateTransforms(
            translationOffset: 10.0
        );

        var worldToFlange =
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 2.0, 3.0)
            );

        var worldToTool =
            RigidTransform.FromTranslation(
                new Vector3D(4.0, 5.0, 6.0)
            );

        var result = new ForwardKinematicsResult(
            jointFrames,
            linkFrames,
            worldToFlange,
            worldToTool
        );

        Assert.Equal(
            JointVector.Count,
            result.WorldToJointFrames.Count
        );

        Assert.Equal(
            JointVector.Count,
            result.WorldToLinkFrames.Count
        );

        Assert.Equal(
            worldToFlange,
            result.WorldToFlange
        );

        Assert.Equal(
            worldToTool,
            result.WorldToTool
        );
    }

    [Fact]
    public void ConstructorCreatesToolPoseFromToolTransform()
    {
        var orientation =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            );

        var worldToTool = new RigidTransform(
            new Vector3D(1.0, 2.0, 3.0),
            orientation
        );

        var result = new ForwardKinematicsResult(
            CreateTransforms(0.0),
            CreateTransforms(10.0),
            RigidTransform.Identity,
            worldToTool
        );

        Assert.Equal(
            worldToTool.Translation,
            result.ToolPose.Position
        );

        Assert.Equal(
            worldToTool.Rotation,
            result.ToolPose.Orientation
        );
    }

    [Fact]
    public void ConstructorCopiesJointFrameCollection()
    {
        var jointFrames = CreateTransforms(0.0);

        var result = new ForwardKinematicsResult(
            jointFrames,
            CreateTransforms(10.0),
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        jointFrames[0] =
            RigidTransform.FromTranslation(
                new Vector3D(100.0, 0.0, 0.0)
            );

        Assert.NotEqual(
            jointFrames[0],
            result.WorldToJointFrames[0]
        );
    }

    [Fact]
    public void ConstructorCopiesLinkFrameCollection()
    {
        var linkFrames = CreateTransforms(0.0);

        var result = new ForwardKinematicsResult(
            CreateTransforms(10.0),
            linkFrames,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        linkFrames[0] =
            RigidTransform.FromTranslation(
                new Vector3D(100.0, 0.0, 0.0)
            );

        Assert.NotEqual(
            linkFrames[0],
            result.WorldToLinkFrames[0]
        );
    }

    [Fact]
    public void ConstructorThrowsForNullJointFrames()
    {
        Assert.Throws<ArgumentNullException>(
            () => new ForwardKinematicsResult(
                null!,
                CreateTransforms(0.0),
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Fact]
    public void ConstructorThrowsForNullLinkFrames()
    {
        Assert.Throws<ArgumentNullException>(
            () => new ForwardKinematicsResult(
                CreateTransforms(0.0),
                null!,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(5)]
    [InlineData(7)]
    public void ConstructorThrowsForInvalidJointFrameCount(
        int count)
    {
        var jointFrames = CreateTransforms(
            count,
            translationOffset: 0.0
        );

        Assert.Throws<ArgumentException>(
            () => new ForwardKinematicsResult(
                jointFrames,
                CreateTransforms(10.0),
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(5)]
    [InlineData(7)]
    public void ConstructorThrowsForInvalidLinkFrameCount(
        int count)
    {
        var linkFrames = CreateTransforms(
            count,
            translationOffset: 0.0
        );

        Assert.Throws<ArgumentException>(
            () => new ForwardKinematicsResult(
                CreateTransforms(10.0),
                linkFrames,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    private static RigidTransform[] CreateTransforms(
        double translationOffset)
    {
        return CreateTransforms(
            JointVector.Count,
            translationOffset
        );
    }

    private static RigidTransform[] CreateTransforms(
        int count,
        double translationOffset)
    {
        return Enumerable
            .Range(0, count)
            .Select(index =>
                RigidTransform.FromTranslation(
                    new Vector3D(
                        translationOffset + index,
                        0.0,
                        0.0
                    )
                )
            )
            .ToArray();
    }
}
