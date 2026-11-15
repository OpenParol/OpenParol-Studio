using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Core.Tests.Geometry;

public sealed class RigidTransformTests
{
    private const int Precision = 12;

    [Fact]
    public void IdentityDoesNotChangePoint()
    {
        var point = new Vector3D(
            1.0,
            2.0,
            3.0
        );

        var result = RigidTransform.Identity.TransformPoint(point);

        AssertVectorApproximatelyEqual(point, result);
    }

    [Fact]
    public void IdentityDoesNotChangeDirection()
    {
        var direction = new Vector3D(
            1.0,
            2.0,
            3.0
        );

        var result =
            RigidTransform.Identity.TransformDirection(direction);

        AssertVectorApproximatelyEqual(direction, result);
    }

    [Fact]
    public void TransformPointAppliesRotationThenTranslation()
    {
        var transform = new RigidTransform(
            new Vector3D(10.0, 20.0, 30.0),
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            )
        );

        var point = new Vector3D(
            1.0,
            0.0,
            0.0
        );

        var result = transform.TransformPoint(point);

        AssertVectorApproximatelyEqual(
            new Vector3D(10.0, 21.0, 30.0),
            result
        );
    }

    [Fact]
    public void TransformDirectionAppliesRotation()
    {
        var transform = new RigidTransform(
            Vector3D.Zero,
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            )
        );

        var direction = new Vector3D(
            1.0,
            0.0,
            0.0
        );

        var result = transform.TransformDirection(direction);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result
        );
    }

    [Fact]
    public void TransformDirectionDoesNotApplyTranslation()
    {
        var transform = new RigidTransform(
            new Vector3D(10.0, 20.0, 30.0),
            QuaternionD.Identity
        );

        var direction = new Vector3D(
            1.0,
            2.0,
            3.0
        );

        var result = transform.TransformDirection(direction);

        AssertVectorApproximatelyEqual(direction, result);
    }

    [Fact]
    public void FromTranslationCreatesTranslationOnlyTransform()
    {
        var translation = new Vector3D(
            1.0,
            2.0,
            3.0
        );

        var transform =
            RigidTransform.FromTranslation(translation);

        Assert.Equal(translation, transform.Translation);
        Assert.Equal(QuaternionD.Identity, transform.Rotation);
    }

    [Fact]
    public void FromRotationCreatesRotationOnlyTransform()
    {
        var rotation = QuaternionD.FromAxisAngle(
            new Vector3D(0.0, 0.0, 1.0),
            Math.PI / 2.0
        );

        var transform =
            RigidTransform.FromRotation(rotation);

        Assert.Equal(Vector3D.Zero, transform.Translation);

        AssertQuaternionApproximatelyEqual(
            rotation,
            transform.Rotation
        );
    }

    [Fact]
    public void FromRotationNormalizesRotation()
    {
        var rotation = new QuaternionD(
            0.0,
            0.0,
            2.0,
            2.0
        );

        var transform =
            RigidTransform.FromRotation(rotation);

        Assert.Equal(Vector3D.Zero, transform.Translation);
        Assert.Equal(1.0, transform.Rotation.Length, Precision);
    }

    [Fact]
    public void FromRotationThrowsForZeroQuaternion()
    {
        var rotation = new QuaternionD(
            0.0,
            0.0,
            0.0,
            0.0
        );

        Assert.Throws<InvalidOperationException>(
            () => RigidTransform.FromRotation(rotation)
        );
    }

    [Fact]
    public void InverseUndoesTransformationOfPoint()
    {
        var transform = new RigidTransform(
            new Vector3D(0.5, -0.2, 1.1),
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.234
            )
        );

        var point = new Vector3D(
            0.3,
            0.7,
            -0.4
        );

        var transformed =
            transform.TransformPoint(point);

        var restored =
            transform.Inverse().TransformPoint(transformed);

        AssertVectorApproximatelyEqual(point, restored);
    }

    [Fact]
    public void InverseUndoesTransformationOfDirection()
    {
        var transform = new RigidTransform(
            new Vector3D(0.5, -0.2, 1.1),
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.234
            )
        );

        var direction = new Vector3D(
            0.3,
            0.7,
            -0.4
        );

        var transformed =
            transform.TransformDirection(direction);

        var restored =
            transform.Inverse().TransformDirection(transformed);

        AssertVectorApproximatelyEqual(direction, restored);
    }

    [Fact]
    public void TransformMultipliedByItsInverseActsAsIdentity()
    {
        var transform = new RigidTransform(
            new Vector3D(0.5, -0.2, 1.1),
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.234
            )
        );

        var combined =
            transform * transform.Inverse();

        var point = new Vector3D(
            0.3,
            0.7,
            -0.4
        );

        var result = combined.TransformPoint(point);

        AssertVectorApproximatelyEqual(point, result);
        Assert.Equal(1.0, combined.Rotation.Length, Precision);
    }

    [Fact]
    public void InverseMultipliedByTransformActsAsIdentity()
    {
        var transform = new RigidTransform(
            new Vector3D(0.5, -0.2, 1.1),
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.234
            )
        );

        var combined =
            transform.Inverse() * transform;

        var point = new Vector3D(
            0.3,
            0.7,
            -0.4
        );

        var result = combined.TransformPoint(point);

        AssertVectorApproximatelyEqual(point, result);
        Assert.Equal(1.0, combined.Rotation.Length, Precision);
    }

    [Fact]
    public void MultiplicationAppliesRightTransformFirst()
    {
        var localTranslation =
            RigidTransform.FromTranslation(
                new Vector3D(1.0, 0.0, 0.0)
            );

        var globalRotation =
            RigidTransform.FromRotation(
                QuaternionD.FromAxisAngle(
                    new Vector3D(0.0, 0.0, 1.0),
                    Math.PI / 2.0
                )
            );

        var combined =
            globalRotation * localTranslation;

        var result =
            combined.TransformPoint(Vector3D.Zero);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result
        );
    }

    [Fact]
    public void ComposedTransformMatchesSequentialApplication()
    {
        var left = new RigidTransform(
            new Vector3D(0.5, 0.2, -0.1),
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 3.0
            )
        );

        var right = new RigidTransform(
            new Vector3D(0.1, -0.4, 0.3),
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 0.0, 0.0),
                Math.PI / 4.0
            )
        );

        var point = new Vector3D(
            0.2,
            0.3,
            0.4
        );

        var sequential =
            left.TransformPoint(
                right.TransformPoint(point)
            );

        var composed =
            (left * right).TransformPoint(point);

        AssertVectorApproximatelyEqual(
            sequential,
            composed
        );
    }

    [Fact]
    public void MultiplicationReturnsNormalizedRotation()
    {
        var first =
            RigidTransform.FromRotation(
                QuaternionD.FromAxisAngle(
                    new Vector3D(1.0, 0.0, 0.0),
                    Math.PI / 3.0
                )
            );

        var second =
            RigidTransform.FromRotation(
                QuaternionD.FromAxisAngle(
                    new Vector3D(0.0, 1.0, 0.0),
                    Math.PI / 4.0
                )
            );

        var result = first * second;

        Assert.Equal(
            1.0,
            result.Rotation.Length,
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

    private static void AssertQuaternionApproximatelyEqual(
        QuaternionD expected,
        QuaternionD actual)
    {
        Assert.Equal(expected.X, actual.X, Precision);
        Assert.Equal(expected.Y, actual.Y, Precision);
        Assert.Equal(expected.Z, actual.Z, Precision);
        Assert.Equal(expected.W, actual.W, Precision);
    }

    [Fact]
    public void InverseOfIdentityIsIdentity()
    {
        var result = RigidTransform.Identity.Inverse();

        Assert.Equal(Vector3D.Zero, result.Translation);

        AssertQuaternionApproximatelyEqual(
            QuaternionD.Identity,
            result.Rotation
        );
    }

    [Fact]
    public void TransformPointThrowsWhenRotationIsZeroQuaternion()
    {
        var transform = new RigidTransform(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        Assert.Throws<InvalidOperationException>(
            () => transform.TransformPoint(Vector3D.Zero)
        );
    }

    [Fact]
    public void InverseThrowsWhenRotationIsZeroQuaternion()
    {
        var transform = new RigidTransform(
            Vector3D.Zero,
            new QuaternionD(
                0.0,
                0.0,
                0.0,
                0.0
            )
        );

        Assert.Throws<InvalidOperationException>(
            () => transform.Inverse()
        );
    }

    [Fact]
    public void TransformsWithEqualComponentsAreEqual()
    {
        var first = new RigidTransform(
            new Vector3D(1.0, 2.0, 3.0),
            QuaternionD.Identity
        );

        var second = new RigidTransform(
            new Vector3D(1.0, 2.0, 3.0),
            QuaternionD.Identity
        );

        Assert.Equal(first, second);
    }

    [Fact]
    public void TransformsWithDifferentComponentsAreNotEqual()
    {
        var first = new RigidTransform(
            new Vector3D(1.0, 2.0, 3.0),
            QuaternionD.Identity
        );

        var second = new RigidTransform(
            new Vector3D(1.0, 2.0, 4.0),
            QuaternionD.Identity
        );

        Assert.NotEqual(first, second);
    }
}
