using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;
using OpenParol.Studio.Kinematics.Forward;
using OpenParol.Studio.Kinematics.Models;

namespace OpenParol.Studio.Kinematics.Tests.Forward;

public sealed class ForwardKinematicsSolverTests
{
    private const int Precision = 12;

    [Fact]
    public void ConstructorThrowsForNullModel()
    {
        Assert.Throws<ArgumentNullException>(
            () => new ForwardKinematicsSolver(null!)
        );
    }

    [Fact]
    public void ZeroPositionsProduceIdentityForIdentityModel()
    {
        var model = CreateIdentityModel();
        var solver = new ForwardKinematicsSolver(model);

        var result = solver.Calculate(ZeroJointVector());

        AssertTransformApproximatelyEqual(
            RigidTransform.Identity,
            result.WorldToFlange
        );

        AssertTransformApproximatelyEqual(
            RigidTransform.Identity,
            result.WorldToTool
        );

        Assert.Equal(
            RobotPose.Identity.Position,
            result.ToolPose.Position
        );

        AssertQuaternionApproximatelyEquivalent(
            RobotPose.Identity.Orientation,
            result.ToolPose.Orientation
        );
    }

    [Fact]
    public void JointRotationRotatesTheCorrespondingLinkFrame()
    {
        var model = CreateIdentityModel();
        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        var rotatedDirection =
            result.WorldToLinkFrames[0]
                .TransformDirection(
                    new Vector3D(1.0, 0.0, 0.0)
                );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            rotatedDirection
        );
    }

    [Fact]
    public void JointFrameDoesNotIncludeItsOwnRotation()
    {
        var model = CreateIdentityModel();
        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertTransformApproximatelyEqual(
            RigidTransform.Identity,
            result.WorldToJointFrames[0]
        );

        var rotatedDirection =
            result.WorldToLinkFrames[0]
                .TransformDirection(
                    new Vector3D(1.0, 0.0, 0.0)
                );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            rotatedDirection
        );
    }

    [Fact]
    public void JointFrameIncludesRotationsOfPreviousJoints()
    {
        var joints = CreateIdentityJoints();

        joints[1] = CreateJoint(
            name: "J2",
            parentToJoint:
                RigidTransform.FromTranslation(
                    new Vector3D(1.0, 0.0, 0.0)
                )
        );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result.WorldToJointFrames[1].Translation
        );

        var jointXAxisInWorld =
            result.WorldToJointFrames[1]
                .TransformDirection(
                    new Vector3D(1.0, 0.0, 0.0)
                );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            jointXAxisInWorld
        );
    }

    [Fact]
    public void ParentToJointIsAppliedBeforeJointRotation()
    {
        var joints = CreateIdentityJoints();

        joints[0] = CreateJoint(
            name: "J1",
            parentToJoint:
                RigidTransform.FromTranslation(
                    new Vector3D(1.0, 0.0, 0.0)
                )
        );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertVectorApproximatelyEqual(
            new Vector3D(1.0, 0.0, 0.0),
            result.WorldToJointFrames[0].Translation
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(1.0, 0.0, 0.0),
            result.WorldToLinkFrames[0].Translation
        );
    }

    [Fact]
    public void PreviousJointRotationAffectsNextJointTranslation()
    {
        var joints = CreateIdentityJoints();

        joints[1] = CreateJoint(
            name: "J2",
            parentToJoint:
                RigidTransform.FromTranslation(
                    new Vector3D(1.0, 0.0, 0.0)
                )
        );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result.WorldToJointFrames[1].Translation
        );
    }

    [Fact]
    public void ZeroOffsetIsAddedToJointPosition()
    {
        var joints = CreateIdentityJoints();

        joints[0] = CreateJoint(
            name: "J1",
            zeroOffset: Math.PI / 2.0
        );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            RigidTransform.Identity
        );

        var solver = new ForwardKinematicsSolver(model);

        var result = solver.Calculate(ZeroJointVector());

        var rotatedDirection =
            result.WorldToLinkFrames[0]
                .TransformDirection(
                    new Vector3D(1.0, 0.0, 0.0)
                );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            rotatedDirection
        );
    }

    [Fact]
    public void BaseTransformAffectsEveryFrame()
    {
        var baseTransform =
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 2.0, 3.0)
            );

        var model = new RobotKinematicModel(
            CreateIdentityJoints(),
            baseTransform,
            RigidTransform.Identity
        );

        var solver = new ForwardKinematicsSolver(model);

        var result = solver.Calculate(ZeroJointVector());

        for (var i = 0; i < JointVector.Count; i++)
        {
            AssertVectorApproximatelyEqual(
                baseTransform.Translation,
                result.WorldToJointFrames[i].Translation
            );

            AssertVectorApproximatelyEqual(
                baseTransform.Translation,
                result.WorldToLinkFrames[i].Translation
            );
        }

        AssertVectorApproximatelyEqual(
            baseTransform.Translation,
            result.WorldToFlange.Translation
        );

        AssertVectorApproximatelyEqual(
            baseTransform.Translation,
            result.WorldToTool.Translation
        );
    }

    [Fact]
    public void FlangeToToolIsAppliedAfterCompleteJointChain()
    {
        var joints = CreateIdentityJoints();

        joints[0] = CreateJoint(
            name: "J1",
            axis: new Vector3D(0.0, 0.0, 1.0)
        );

        var flangeToTool =
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 0.0, 0.0)
            );

        var model = new RobotKinematicModel(
            joints,
            RigidTransform.Identity,
            flangeToTool
        );

        var solver = new ForwardKinematicsSolver(model);

        var positions = new JointVector(
            Math.PI / 2.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var result = solver.Calculate(positions);

        AssertVectorApproximatelyEqual(
            Vector3D.Zero,
            result.WorldToFlange.Translation
        );

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result.WorldToTool.Translation
        );
    }

    [Fact]
    public void ToolPoseMatchesWorldToTool()
    {
        var model = new RobotKinematicModel(
            CreateIdentityJoints(),
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 2.0, 3.0)
            ),
            RigidTransform.FromTranslation(
                new Vector3D(0.0, 0.0, 0.5)
            )
        );

        var solver = new ForwardKinematicsSolver(model);

        var result = solver.Calculate(ZeroJointVector());

        AssertVectorApproximatelyEqual(
            result.WorldToTool.Translation,
            result.ToolPose.Position
        );

        AssertQuaternionApproximatelyEquivalent(
            result.WorldToTool.Rotation,
            result.ToolPose.Orientation
        );
    }

    [Fact]
    public void CalculateThrowsWhenJointPositionIsBelowMinimum()
    {
        var solver =
            new ForwardKinematicsSolver(
                CreateIdentityModel()
            );

        var positions = new JointVector(
            -Math.PI - 0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );

        var exception =
            Assert.Throws<ArgumentOutOfRangeException>(
                () => solver.Calculate(positions)
            );

        Assert.Equal(
            "jointPositions",
            exception.ParamName
        );

        Assert.Contains(
            "J1",
            exception.Message
        );
    }

    [Fact]
    public void CalculateThrowsWhenJointPositionIsAboveMaximum()
    {
        var solver =
            new ForwardKinematicsSolver(
                CreateIdentityModel()
            );

        var positions = new JointVector(
            0.0,
            0.0,
            0.0,
            Math.PI + 0.1,
            0.0,
            0.0
        );

        var exception =
            Assert.Throws<ArgumentOutOfRangeException>(
                () => solver.Calculate(positions)
            );

        Assert.Equal(
            "jointPositions",
            exception.ParamName
        );

        Assert.Contains(
            "J4",
            exception.Message
        );
    }

    [Fact]
    public void CalculateAcceptsPositionsExactlyAtLimits()
    {
        var solver =
            new ForwardKinematicsSolver(
                CreateIdentityModel()
            );

        var positions = new JointVector(
            -Math.PI,
            Math.PI,
            -Math.PI,
            Math.PI,
            -Math.PI,
            Math.PI
        );

        var result = solver.Calculate(positions);

        Assert.Equal(
            JointVector.Count,
            result.WorldToJointFrames.Count
        );

        Assert.Equal(
            JointVector.Count,
            result.WorldToLinkFrames.Count
        );
    }

    private static RobotKinematicModel CreateIdentityModel()
    {
        return new RobotKinematicModel(
            CreateIdentityJoints(),
            RigidTransform.Identity,
            RigidTransform.Identity
        );
    }

    private static RevoluteJointDefinition[] CreateIdentityJoints()
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
        string name,
        Vector3D? axis = null,
        RigidTransform? parentToJoint = null,
        double zeroOffset = 0.0)
    {
        return new RevoluteJointDefinition(
            name,
            axis ?? new Vector3D(0.0, 0.0, 1.0),
            parentToJoint ?? RigidTransform.Identity,
            new JointPositionLimits(
                -Math.PI,
                Math.PI
            ),
            zeroOffset
        );
    }

    private static JointVector ZeroJointVector()
    {
        return new JointVector(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        );
    }

    private static void AssertTransformApproximatelyEqual(
        RigidTransform expected,
        RigidTransform actual)
    {
        AssertVectorApproximatelyEqual(
            expected.Translation,
            actual.Translation
        );

        AssertQuaternionApproximatelyEquivalent(
            expected.Rotation,
            actual.Rotation
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
