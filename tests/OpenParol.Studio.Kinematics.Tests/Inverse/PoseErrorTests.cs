using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Inverse;

namespace OpenParol.Studio.Kinematics.Tests.Inverse;

public sealed class PoseErrorTests
{
    private const int Precision = 12;

    [Fact]
    public void EqualPosesProduceZeroError()
    {
        var pose = new RobotPose(
            new Vector3D(0.2, 0.1, 0.3),
            QuaternionD.Identity
        );

        var result = PoseError.Calculate(
            pose,
            pose
        );

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result.Position
        );

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result.Orientation
        );

        Assert.Equal(
            0.0,
            result.PositionMagnitude,
            Precision
        );

        Assert.Equal(
            0.0,
            result.OrientationMagnitude,
            Precision
        );
    }

    [Fact]
    public void PositionErrorPointsFromCurrentToTarget()
    {
        var current = new RobotPose(
            new Vector3D(0.20, 0.10, 0.30),
            QuaternionD.Identity
        );

        var target = new RobotPose(
            new Vector3D(0.25, 0.08, 0.32),
            QuaternionD.Identity
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.05, -0.02, 0.02),
            result.Position
        );

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result.Orientation
        );
    }

    [Fact]
    public void OrientationErrorReturnsRotationAroundZAxis()
    {
        var current = new RobotPose(
            Vector3D.Zero,
            QuaternionD.Identity
        );

        var target = new RobotPose(
            Vector3D.Zero,
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            )
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(
                0.0,
                0.0,
                Math.PI / 2.0
            ),
            result.Orientation
        );

        Assert.Equal(
            Math.PI / 2.0,
            result.OrientationMagnitude,
            Precision
        );
    }

    [Fact]
    public void OrientationErrorUsesWorldCoordinates()
    {
        var currentOrientation =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            );

        var additionalRotation =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 0.0, 0.0),
                Math.PI / 4.0
            );

        var targetOrientation =
            additionalRotation * currentOrientation;

        var current = new RobotPose(
            Vector3D.Zero,
            currentOrientation
        );

        var target = new RobotPose(
            Vector3D.Zero,
            targetOrientation
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(
                Math.PI / 4.0,
                0.0,
                0.0
            ),
            result.Orientation
        );
    }

    [Fact]
    public void OrientationErrorChoosesShortestRotation()
    {
        var current = new RobotPose(
            Vector3D.Zero,
            QuaternionD.Identity
        );

        var target = new RobotPose(
            Vector3D.Zero,
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                350.0 * Math.PI / 180.0
            )
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(
                0.0,
                0.0,
                -10.0 * Math.PI / 180.0
            ),
            result.Orientation
        );
    }

    [Fact]
    public void OppositeQuaternionSignsProduceZeroOrientationError()
    {
        var orientation =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.2
            );

        var oppositeOrientation = new QuaternionD(
            -orientation.X,
            -orientation.Y,
            -orientation.Z,
            -orientation.W
        );

        var current = new RobotPose(
            Vector3D.Zero,
            orientation
        );

        var target = new RobotPose(
            Vector3D.Zero,
            oppositeOrientation
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result.Orientation
        );
    }

    [Fact]
    public void CalculateNormalizesInputOrientations()
    {
        var unitTarget =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 1.0, 0.0),
                Math.PI / 3.0
            );

        var scaledTarget = new QuaternionD(
            unitTarget.X * 4.0,
            unitTarget.Y * 4.0,
            unitTarget.Z * 4.0,
            unitTarget.W * 4.0
        );

        var current = new RobotPose(
            Vector3D.Zero,
            QuaternionD.Identity
        );

        var target = new RobotPose(
            Vector3D.Zero,
            scaledTarget
        );

        var result = PoseError.Calculate(
            current,
            target
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(
                0.0,
                Math.PI / 3.0,
                0.0
            ),
            result.Orientation
        );
    }

    [Fact]
    public void CalculateThrowsForZeroCurrentOrientation()
    {
        var current = new RobotPose(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        var target = RobotPose.Identity;

        Assert.Throws<InvalidOperationException>(
            () => PoseError.Calculate(
                current,
                target
            )
        );
    }

    [Fact]
    public void CalculateThrowsForZeroTargetOrientation()
    {
        var current = RobotPose.Identity;

        var target = new RobotPose(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        Assert.Throws<InvalidOperationException>(
            () => PoseError.Calculate(
                current,
                target
            )
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
