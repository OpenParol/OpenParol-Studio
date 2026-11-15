using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Core.Tests.Geometry;

public sealed class Vector3DTests
{
    [Fact]
    public void ZeroHasAllComponentsEqualToZero()
    {
        Assert.Equal(0.0, Vector3D.Zero.X);
        Assert.Equal(0.0, Vector3D.Zero.Y);
        Assert.Equal(0.0, Vector3D.Zero.Z);
    }

    [Fact]
    public void ZeroHasZeroLength()
    {
        Assert.Equal(0.0, Vector3D.Zero.Length);
        Assert.Equal(0.0, Vector3D.Zero.LengthSquared);
    }

    [Fact]
    public void LengthReturnsEuclideanLength()
    {
        var vector = new Vector3D(3.0, 4.0, 0.0);

        Assert.Equal(5.0, vector.Length);
    }

    [Fact]
    public void LengthSquaredReturnsSquaredEuclideanLength()
    {
        var vector = new Vector3D(3.0, 4.0, 0.0);

        Assert.Equal(25.0, vector.LengthSquared);
    }

    [Fact]
    public void AdditionAddsCorrespondingComponents()
    {
        var left = new Vector3D(1.0, 2.0, 3.0);
        var right = new Vector3D(4.0, 5.0, 6.0);

        var result = left + right;

        Assert.Equal(new Vector3D(5.0, 7.0, 9.0), result);
    }

    [Fact]
    public void SubtractionSubtractsCorrespondingComponents()
    {
        var left = new Vector3D(4.0, 5.0, 6.0);
        var right = new Vector3D(1.0, 2.0, 3.0);

        var result = left - right;

        Assert.Equal(new Vector3D(3.0, 3.0, 3.0), result);
    }

    [Fact]
    public void RightScalarMultiplicationMultipliesEveryComponent()
    {
        var vector = new Vector3D(1.0, 2.0, 3.0);

        var result = vector * 2.0;

        Assert.Equal(new Vector3D(2.0, 4.0, 6.0), result);
    }

    [Fact]
    public void LeftScalarMultiplicationMultipliesEveryComponent()
    {
        var vector = new Vector3D(1.0, 2.0, 3.0);

        var result = 2.0 * vector;

        Assert.Equal(new Vector3D(2.0, 4.0, 6.0), result);
    }

    [Fact]
    public void MultiplicationByZeroReturnsZeroVector()
    {
        var vector = new Vector3D(1.0, -2.0, 3.0);

        var result = vector * 0.0;

        Assert.Equal(Vector3D.Zero, result);
    }

    [Fact]
    public void DotReturnsScalarProduct()
    {
        var left = new Vector3D(1.0, 2.0, 3.0);
        var right = new Vector3D(4.0, 5.0, 6.0);

        var result = Vector3D.Dot(left, right);

        Assert.Equal(32.0, result);
    }

    [Fact]
    public void DotOfPerpendicularVectorsReturnsZero()
    {
        var x = new Vector3D(1.0, 0.0, 0.0);
        var y = new Vector3D(0.0, 1.0, 0.0);

        var result = Vector3D.Dot(x, y);

        Assert.Equal(0.0, result);
    }

    [Fact]
    public void CrossOfXAxisAndYAxisReturnsZAxis()
    {
        var x = new Vector3D(1.0, 0.0, 0.0);
        var y = new Vector3D(0.0, 1.0, 0.0);
        var z = new Vector3D(0.0, 0.0, 1.0);

        var result = Vector3D.Cross(x, y);

        Assert.Equal(z, result);
    }

    [Fact]
    public void CrossOfYAxisAndZAxisReturnsXAxis()
    {
        var x = new Vector3D(1.0, 0.0, 0.0);
        var y = new Vector3D(0.0, 1.0, 0.0);
        var z = new Vector3D(0.0, 0.0, 1.0);

        var result = Vector3D.Cross(y, z);

        Assert.Equal(x, result);
    }

    [Fact]
    public void CrossOfZAxisAndXAxisReturnsYAxis()
    {
        var x = new Vector3D(1.0, 0.0, 0.0);
        var y = new Vector3D(0.0, 1.0, 0.0);
        var z = new Vector3D(0.0, 0.0, 1.0);

        var result = Vector3D.Cross(z, x);

        Assert.Equal(y, result);
    }

    [Fact]
    public void ReversingCrossProductNegatesResult()
    {
        var x = new Vector3D(1.0, 0.0, 0.0);
        var y = new Vector3D(0.0, 1.0, 0.0);
        var negativeZ = new Vector3D(0.0, 0.0, -1.0);

        var result = Vector3D.Cross(y, x);

        Assert.Equal(negativeZ, result);
    }

    [Fact]
    public void CrossProductIsPerpendicularToBothInputVectors()
    {
        var left = new Vector3D(1.0, 2.0, 3.0);
        var right = new Vector3D(4.0, 5.0, 6.0);

        var cross = Vector3D.Cross(left, right);

        Assert.Equal(0.0, Vector3D.Dot(cross, left), precision: 12);
        Assert.Equal(0.0, Vector3D.Dot(cross, right), precision: 12);
    }

    [Fact]
    public void NormalizeReturnsUnitVector()
    {
        var vector = new Vector3D(3.0, 4.0, 0.0);

        var result = vector.Normalize();

        Assert.Equal(0.6, result.X, precision: 12);
        Assert.Equal(0.8, result.Y, precision: 12);
        Assert.Equal(0.0, result.Z, precision: 12);
        Assert.Equal(1.0, result.Length, precision: 12);
    }

    [Fact]
    public void NormalizePreservesVectorDirection()
    {
        var vector = new Vector3D(2.0, -4.0, 6.0);

        var normalized = vector.Normalize();

        Assert.Equal(
            vector.X / vector.Length,
            normalized.X,
            precision: 12);

        Assert.Equal(
            vector.Y / vector.Length,
            normalized.Y,
            precision: 12);

        Assert.Equal(
            vector.Z / vector.Length,
            normalized.Z,
            precision: 12);
    }

    [Fact]
    public void NormalizeDoesNotModifyUnitVector()
    {
        var vector = new Vector3D(1.0, 0.0, 0.0);

        var result = vector.Normalize();

        Assert.Equal(vector, result);
    }

    [Fact]
    public void NormalizeThrowsForZeroVector()
    {
        Assert.Throws<InvalidOperationException>(
            () => Vector3D.Zero.Normalize());
    }

    [Fact]
    public void UnaryNegationNegatesEveryComponent()
    {
        var vector = new Vector3D(
            1.0,
            -2.0,
            3.0
        );

        var result = -vector;

        Assert.Equal(
            new Vector3D(-1.0, 2.0, -3.0),
            result
        );
    }

    [Fact]
    public void UnaryNegationOfZeroReturnsZero()
    {
        var result = -Vector3D.Zero;

        Assert.Equal(Vector3D.Zero, result);
    }
}
