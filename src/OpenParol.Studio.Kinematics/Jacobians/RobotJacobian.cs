using System.Collections.ObjectModel;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Jacobians;

/// <summary>
/// Represents the six-column geometric Jacobian of a six-axis robot.
/// </summary>
public sealed class RobotJacobian
{
    private readonly ReadOnlyCollection<JacobianColumn> _columns;

    public IReadOnlyList<JacobianColumn> Columns => _columns;

    public JacobianColumn this[int index] => _columns[index];

    public RobotJacobian(
        IEnumerable<JacobianColumn> columns)
    {
        ArgumentNullException.ThrowIfNull(columns);

        var columnArray = columns.ToArray();

        if (columnArray.Length != JointVector.Count)
        {
            throw new ArgumentException(
                $"A robot Jacobian must contain exactly "
                    + $"{JointVector.Count} columns.",
                nameof(columns)
            );
        }

        _columns = Array.AsReadOnly(columnArray);
    }
}
