namespace OpenParol.Studio.Core.Robotics;

/// <summary>
/// Represents a vector containing one value for each of the six robot joints.
/// </summary>
/// <remarks>
/// The unit of each value depends on the context in which the vector is used.
/// For joint positions, values are expressed in radians.
/// </remarks>
public readonly record struct JointVector(
    double J1,
    double J2,
    double J3,
    double J4,
    double J5,
    double J6)
{
    public const int Count = 6;

    public double this[int index] => index switch
    {
        0 => J1,
        1 => J2,
        2 => J3,
        3 => J4,
        4 => J5,
        5 => J6,
        _ => throw new ArgumentOutOfRangeException(nameof(index), index,
                    $"Joint index must be between 0 and {Count - 1}.")
    };
}
