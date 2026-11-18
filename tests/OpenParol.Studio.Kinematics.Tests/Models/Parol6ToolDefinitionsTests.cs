using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Models;

public sealed class Parol6ToolDefinitionsTests
{
    private const int Precision = 12;

    [Fact]
    public void FlangeToolIsIdentity()
    {
        Assert.Equal(
            RigidTransform.Identity,
            Parol6ToolDefinitions.Flange
        );
    }

    [Fact]
    public void PneumaticGripperHasExpectedTranslation()
    {
        var tool =
            Parol6ToolDefinitions.PneumaticGripper;

        Assert.Equal(
            -0.04525,
            tool.Translation.X,
            Precision
        );

        Assert.Equal(
            0.0,
            tool.Translation.Y,
            Precision
        );

        Assert.Equal(
            0.0118,
            tool.Translation.Z,
            Precision
        );
    }

    [Fact]
    public void PneumaticGripperHasIdentityEquivalentOrientation()
    {
        var tool =
        Parol6ToolDefinitions.PneumaticGripper;

        AssertVectorApproximatelyEqual(
            new Vector3D(1.0, 0.0, 0.0),
            tool.TransformDirection(
                new Vector3D(1.0, 0.0, 0.0)
            )
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            tool.TransformDirection(
                new Vector3D(0.0, 1.0, 0.0)
            )
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 0.0, 1.0),
            tool.TransformDirection(
                new Vector3D(0.0, 0.0, 1.0)
            )
        );

        Assert.Equal(
            1.0,
            tool.Rotation.Length,
            Precision
        );
    }

    private static void AssertVectorApproximatelyEqual(
        Vector3D expected,
        Vector3D actual)
    {
        Assert.Equal(expected.X, actual.X, Precision);
        Assert.Equal(expected.Y, actual.Y, Precision);
        Assert.Equal(expected.Z, actual.Z, Precision);
    }
}
