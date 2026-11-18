using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Models;

public sealed class Parol6KinematicModelFactoryTests
{
    private const int Precision = 12;

    [Fact]
    public void CreateReturnsSixJointModel()
    {
        var model = CreateModel();

        Assert.Equal(
            JointVector.Count,
            model.Joints.Count
        );
    }

    [Fact]
    public void CreateUsesExpectedJointNames()
    {
        var model = CreateModel();

        var names = model.Joints
            .Select(joint => joint.Name)
            .ToArray();

        Assert.Equal(
            ["J1", "J2", "J3", "J4", "J5", "J6"],
            names
        );
    }

    [Fact]
    public void CreateUsesIdentityBaseTransform()
    {
        var model = CreateModel();

        Assert.Equal(
            RigidTransform.Identity,
            model.BaseTransform
        );
    }

    [Fact]
    public void CreateUsesProvidedToolTransform()
    {
        var tool = RigidTransform.FromTranslation(
            new Vector3D(0.1, 0.2, 0.3)
        );

        var model =
            Parol6KinematicModelFactory.Create(tool);

        Assert.Equal(tool, model.FlangeToTool);
    }

    [Fact]
    public void CreateUsesExpectedLastLinkToFlangeTransform()
    {
        var model = CreateModel();

        Assert.Equal(
            0.0,
            model.LastLinkToFlange.Translation.X,
            Precision
        );

        Assert.Equal(
            0.0,
            model.LastLinkToFlange.Translation.Y,
            Precision
        );

        Assert.Equal(
            0.0505,
            model.LastLinkToFlange.Translation.Z,
            Precision
        );

        Assert.Equal(
            QuaternionD.Identity,
            model.LastLinkToFlange.Rotation
        );
    }

    [Theory]
    [InlineData(0, -1.7, 1.7)]
    [InlineData(1, -0.98, 1.0)]
    [InlineData(2, -1.3, 2.0)]
    [InlineData(3, -2.0, 2.0)]
    [InlineData(4, -2.1, 2.1)]
    [InlineData(5, -3.1, 3.1)]
    public void CreateUsesExpectedJointLimits(
        int index,
        double minimum,
        double maximum)
    {
        var model = CreateModel();
        var limits = model.Joints[index].PositionLimits;

        Assert.Equal(minimum, limits.Minimum, Precision);
        Assert.Equal(maximum, limits.Maximum, Precision);
    }

    [Fact]
    public void CreateUsesNegativeLocalZAxisForJ6()
    {
        var model = CreateModel();
        var axis = model.Joints[5].Axis;

        Assert.Equal(0.0, axis.X, Precision);
        Assert.Equal(0.0, axis.Y, Precision);
        Assert.Equal(-1.0, axis.Z, Precision);
    }

    [Fact]
    public void CreateUsesExpectedJ2Translation()
    {
        var model = CreateModel();
        var translation =
            model.Joints[1].ParentToJoint.Translation;

        Assert.Equal(
            0.0234207210610365,
            translation.X,
            Precision
        );

        Assert.Equal(
            0.0,
            translation.Y,
            Precision
        );

        Assert.Equal(
            0.1105,
            translation.Z,
            Precision
        );
    }

    [Fact]
    public void CreateUsesExpectedJ3Translation()
    {
        var model = CreateModel();
        var translation =
            model.Joints[2].ParentToJoint.Translation;

        Assert.Equal(0.0, translation.X, Precision);
        Assert.Equal(-0.18, translation.Y, Precision);
        Assert.Equal(0.0, translation.Z, Precision);
    }

    [Fact]
    public void CreateUsesExpectedJ4Translation()
    {
        var model = CreateModel();
        var translation =
            model.Joints[3].ParentToJoint.Translation;

        Assert.Equal(0.0435, translation.X, Precision);
        Assert.Equal(0.0, translation.Y, Precision);
        Assert.Equal(0.0, translation.Z, Precision);
    }

    [Fact]
    public void CreateUsesExpectedJ5Translation()
    {
        var model = CreateModel();
        var translation =
            model.Joints[4].ParentToJoint.Translation;

        Assert.Equal(0.0, translation.X, Precision);
        Assert.Equal(0.0, translation.Y, Precision);
        Assert.Equal(-0.177, translation.Z, Precision);
    }

    private static RobotKinematicModel CreateModel()
    {
        return Parol6KinematicModelFactory.Create(
            Parol6ToolDefinitions.PneumaticGripper
        );
    }
}
