using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Models;

public sealed class RobotKinematicModelTests
{
    [Fact]
    public void ConstructorStoresModelData()
    {
        var joints = CreateSixJoints();

        var baseTransform =
            RigidTransform.FromTranslation(
            new Vector3D(1.0, 2.0, 3.0)
        );

        var flangeToTool =
            RigidTransform.FromTranslation(
            new Vector3D(0.0, 0.0, 0.1)
        );

        var model = new RobotKinematicModel(
            joints,
            baseTransform,
            flangeToTool
        );

        Assert.Equal(JointVector.Count, model.Joints.Count);
        Assert.Equal(baseTransform, model.BaseTransform);
        Assert.Equal(flangeToTool, model.FlangeToTool);
    }

    [Fact]
    public void ConstructorCopiesJointCollection()
    {
        var joints = CreateSixJoints();

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        joints[0] = CreateJoint("Changed");

        Assert.Equal("J1", model.Joints[0].Name);
    }

    [Fact]
    public void ConstructorThrowsForNullJointCollection()
    {
        Assert.Throws<ArgumentNullException>(
            () => new RobotKinematicModel(
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
    public void ConstructorThrowsForInvalidJointCount(
        int count)
    {
        var joints = Enumerable
            .Range(1, count)
            .Select(index => CreateJoint($"J{index}"));

        Assert.Throws<ArgumentException>(
            () => new RobotKinematicModel(
                joints,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Fact]
    public void ConstructorThrowsForNullJoint()
    {
        var joints = CreateSixJoints();
        joints[3] = null!;

        Assert.Throws<ArgumentException>(
            () => new RobotKinematicModel(
                joints,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Fact]
    public void ConstructorThrowsForDuplicateJointName()
    {
        var joints = CreateSixJoints();
        joints[5] = CreateJoint("J1");

        Assert.Throws<ArgumentException>(
            () => new RobotKinematicModel(
                joints,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    [Fact]
    public void DuplicateNameCheckIgnoresLetterCase()
    {
        var joints = CreateSixJoints();
        joints[5] = CreateJoint("j1");

        Assert.Throws<ArgumentException>(
            () => new RobotKinematicModel(
                joints,
                RigidTransform.Identity,
                RigidTransform.Identity
            )
        );
    }

    private static RevoluteJointDefinition[] CreateSixJoints()
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
        string name)
    {
        return new RevoluteJointDefinition(
            name,
            new Vector3D(0.0, 0.0, 1.0),
            RigidTransform.Identity,
            new JointPositionLimits(-Math.PI, Math.PI)
        );
    }
}
