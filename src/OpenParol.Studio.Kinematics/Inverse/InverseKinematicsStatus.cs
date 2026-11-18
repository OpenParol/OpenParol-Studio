namespace OpenParol.Studio.Kinematics.Inverse;

/// <summary>
/// Describes the outcome of an inverse-kinematics calculation.
/// </summary>
public enum InverseKinematicsStatus
{
    Converged,
    /// <summary>
    /// The maximum number of iterations has been reached.
    /// </summary>
    MaximumIterationsReached,
    /// <summary>
    /// A joint limit has been reached.
    /// </summary>
    JointLimitReached
}
