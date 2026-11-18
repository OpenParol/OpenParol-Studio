using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Forward;

public sealed class Parol6ForwardKinematicsTests
{
    private const int Precision = 12;

    [Fact]
    public void ZeroConfigurationProducesExpectedPneumaticToolPose()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var solver =
            new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertVectorApproximatelyEqual(
            new Vector3D(
                0.2627207210610365,
                0.0,
                0.28875
            ),
            result.ToolPose.Position
        );

        AssertQuaternionApproximatelyEquivalent(
            new QuaternionD(
                Math.Sqrt(0.5),
                0.0,
                Math.Sqrt(0.5),
                0.0
            ),
            result.ToolPose.Orientation
        );
    }

    [Fact]
    public void ZeroConfigurationProducesExpectedJointOrigins()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var solver =
            new ForwardKinematicsSolver(model);

        var result = solver.Calculate(
            new JointVector(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        var expectedOrigins = new[]
        {
            new Vector3D(
                0.0,
                0.0,
                0.0
            ),
            new Vector3D(
                0.0234207210610365,
                0.0,
                0.1105
            ),
            new Vector3D(
                0.0234207210610365,
                0.0,
                0.2905
            ),
            new Vector3D(
                0.0234207210610365,
                0.0,
                0.334
            ),
            new Vector3D(
                0.2004207210610365,
                0.0,
                0.334
            ),
            new Vector3D(
                0.2004207210610365,
                0.0,
                0.334
            )
        };

        for (var i = 0; i < JointVector.Count; i++)
        {
            AssertVectorApproximatelyEqual(
                expectedOrigins[i],
                result.WorldToJointFrames[i].Translation
            );
        }
    }

    [Fact]
    public void ZeroConfigurationProducesExpectedFlangePosition()
    {
        var model =
            Parol6KinematicModelFactory.Create(
                Parol6ToolDefinitions.PneumaticGripper
            );

        var solver =
            new ForwardKinematicsSolver(model);

        var result = solver.Calculate(
            new JointVector(
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(
                0.2509207210610365,
                0.0,
                0.334
            ),
            result.WorldToFlange.Translation
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

    private static void AssertQuaternionApproximatelyEquivalent(
        QuaternionD expected,
        QuaternionD actual)
    {
        var directDifference =
            ComponentDifference(expected, actual);

        var negatedDifference =
            ComponentDifference(
                expected,
                new QuaternionD(
                    -actual.X,
                    -actual.Y,
                    -actual.Z,
                    -actual.W
                )
            );

        Assert.True(
            Math.Min(directDifference, negatedDifference)
                < Math.Pow(10.0, -Precision),
            $"Expected orientation equivalent to {expected}, "
                + $"but received {actual}."
        );
    }

    private static double ComponentDifference(
        QuaternionD left,
        QuaternionD right)
    {
        return Math.Max(
            Math.Max(
                Math.Abs(left.X - right.X),
                Math.Abs(left.Y - right.Y)
            ),
            Math.Max(
                Math.Abs(left.Z - right.Z),
                Math.Abs(left.W - right.W)
            )
        );
    }
}
