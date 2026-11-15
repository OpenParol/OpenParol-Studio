using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Core.Tests.Geometry;

public sealed class QuaternionDTests
{
    private const int Precision = 12;

    [Fact]
    public void IdentityHasExpectedComponents()
    {
        Assert.Equal(0.0, QuaternionD.Identity.X);
        Assert.Equal(0.0, QuaternionD.Identity.Y);
        Assert.Equal(0.0, QuaternionD.Identity.Z);
        Assert.Equal(1.0, QuaternionD.Identity.W);
    }

    [Fact]
    public void IdentityHasUnitLength()
    {
        Assert.Equal(1.0, QuaternionD.Identity.Length);
        Assert.Equal(1.0, QuaternionD.Identity.LengthSquared);
    }

    [Fact]
    public void LengthSquaredReturnsSumOfSquaredComponents()
    {
        var quaternion = new QuaternionD(1.0, 2.0, 3.0, 4.0);

        Assert.Equal(30.0, quaternion.LengthSquared);
    }

    [Fact]
    public void LengthReturnsSquareRootOfLengthSquared()
    {
        var quaternion = new QuaternionD(0.0, 0.0, 3.0, 4.0);

        Assert.Equal(5.0, quaternion.Length);
    }

    [Fact]
    public void ConjugateNegatesVectorComponent()
    {
        var quaternion = new QuaternionD(1.0, 2.0, 3.0, 4.0);

        var result = quaternion.Conjugate();

        Assert.Equal(
            new QuaternionD(-1.0, -2.0, -3.0, 4.0),
            result);
    }

    [Fact]
    public void NormalizeReturnsUnitQuaternion()
    {
        var quaternion = new QuaternionD(0.0, 0.0, 3.0, 4.0);

        var result = quaternion.Normalize();

        Assert.Equal(0.0, result.X, Precision);
        Assert.Equal(0.0, result.Y, Precision);
        Assert.Equal(0.6, result.Z, Precision);
        Assert.Equal(0.8, result.W, Precision);
        Assert.Equal(1.0, result.Length, Precision);
    }

    [Fact]
    public void NormalizeDoesNotModifyAlreadyNormalizedQuaternion()
    {
        var result = QuaternionD.Identity.Normalize();

        Assert.Equal(QuaternionD.Identity, result);
    }

    [Fact]
    public void NormalizeThrowsForZeroQuaternion()
    {
        var quaternion = new QuaternionD(0.0, 0.0, 0.0, 0.0);

        Assert.Throws<InvalidOperationException>(
            () => quaternion.Normalize());
    }

    [Fact]
    public void MultiplicationByIdentityReturnsOriginalQuaternion()
    {
        var quaternion =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 3.0);

        var rightResult = quaternion * QuaternionD.Identity;
        var leftResult = QuaternionD.Identity * quaternion;

        AssertQuaternionApproximatelyEqual(quaternion, rightResult);
        AssertQuaternionApproximatelyEqual(quaternion, leftResult);
    }

    [Fact]
    public void MultiplicationByConjugateReturnsIdentityForUnitQuaternion()
    {
        var quaternion =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                Math.PI / 3.0);

        var result = quaternion * quaternion.Conjugate();

        AssertQuaternionApproximatelyEqual(
            QuaternionD.Identity,
            result);
    }

    [Fact]
    public void FromAxisAngleReturnsIdentityForZeroAngle()
    {
        var result =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 0.0, 0.0),
                0.0);

        AssertQuaternionApproximatelyEqual(
            QuaternionD.Identity,
            result);
    }

    [Fact]
    public void FromAxisAngleNormalizesInputAxis()
    {
        var result =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 10.0),
                Math.PI);

        Assert.Equal(0.0, result.X, Precision);
        Assert.Equal(0.0, result.Y, Precision);
        Assert.Equal(1.0, result.Z, Precision);
        Assert.Equal(0.0, result.W, Precision);
        Assert.Equal(1.0, result.Length, Precision);
    }

    [Fact]
    public void FromAxisAngleThrowsForZeroAxis()
    {
        Assert.Throws<ArgumentException>(
            () => QuaternionD.FromAxisAngle(
                Vector3D.Zero,
                Math.PI / 2.0));
    }

    [Fact]
    public void IdentityDoesNotRotateVector()
    {
        var vector = new Vector3D(1.0, 2.0, 3.0);

        var result = QuaternionD.Identity.Rotate(vector);

        AssertVectorApproximatelyEqual(vector, result);
    }

    [Fact]
    public void NinetyDegreeRotationAroundZMapsXAxisToYAxis()
    {
        var rotation =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0);

        var result =
            rotation.Rotate(
                new Vector3D(1.0, 0.0, 0.0));

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result);
    }

    [Fact]
    public void NinetyDegreeRotationAroundXMapsYAxisToZAxis()
    {
        var rotation =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 0.0, 0.0),
                Math.PI / 2.0);

        var result =
            rotation.Rotate(
                new Vector3D(0.0, 1.0, 0.0));

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 0.0, 1.0),
            result);
    }

    [Fact]
    public void RotationPreservesVectorLength()
    {
        var rotation =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 2.0, 3.0),
                1.234);

        var vector = new Vector3D(4.0, -2.0, 7.0);

        var result = rotation.Rotate(vector);

        Assert.Equal(
            vector.Length,
            result.Length,
            Precision);
    }

    [Fact]
    public void QuaternionMultiplicationComposesRotationsRightToLeft()
    {
        var rotateAroundX =
            QuaternionD.FromAxisAngle(
                new Vector3D(1.0, 0.0, 0.0),
                Math.PI / 2.0);

        var rotateAroundZ =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0);

        var combined = rotateAroundZ * rotateAroundX;

        var result =
            combined.Rotate(
                new Vector3D(0.0, 1.0, 0.0));

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 0.0, 1.0),
            result);
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

    private static void AssertVectorApproximatelyEqual(
        Vector3D expected,
        Vector3D actual)
    {
        Assert.Equal(expected.X, actual.X, Precision);
        Assert.Equal(expected.Y, actual.Y, Precision);
        Assert.Equal(expected.Z, actual.Z, Precision);
    }

    [Fact]
    public void RotateNormalizesQuaternionBeforeApplyingRotation()
    {
        var normalizedRotation =
            QuaternionD.FromAxisAngle(
                new Vector3D(0.0, 0.0, 1.0),
                Math.PI / 2.0
            );

        var scaledRotation = new QuaternionD(
            normalizedRotation.X * 5.0,
            normalizedRotation.Y * 5.0,
            normalizedRotation.Z * 5.0,
            normalizedRotation.W * 5.0
        );

        var vector = new Vector3D(
            1.0,
            0.0,
            0.0
        );

        var result = scaledRotation.Rotate(vector);

        AssertVectorApproximatelyEqual(
            new Vector3D(0.0, 1.0, 0.0),
            result
        );
    }

    [Fact]
    public void RotateThrowsForZeroQuaternion()
    {
        var quaternion = new QuaternionD(
            0.0,
            0.0,
            0.0,
            0.0
        );

        var vector = new Vector3D(
            1.0,
            0.0,
            0.0
        );

        Assert.Throws<InvalidOperationException>(
            () => quaternion.Rotate(vector)
        );
    }
}
