using OpenParol.Studio.Core.Geometry;

namespace OpenParol.Studio.Kinematics.Models;

/// <summary>
/// Provides predefined tool transformations for the PAROL6 robot.
/// </summary>
public static class Parol6ToolDefinitions
{
    /// <summary>
    /// Gets a transform representing a tool frame coincident
    /// with the mechanical flange.
    /// </summary>
    public static RigidTransform Flange { get; } =
        RigidTransform.Identity;

    /// <summary>
    /// Gets the transform from the mechanical flange to the operational
    /// TCP of the pneumatic gripper.
    /// </summary>
    public static RigidTransform PneumaticGripper { get; } =
        CreatePneumaticGripper();

    private static RigidTransform CreatePneumaticGripper()
    {
        var flangeToGripper = new RigidTransform(
            new Vector3D(
                0.0,
                0.0,
                -0.015
            ),
            QuaternionD.FromRollPitchYaw(
                0.0,
                Math.PI,
                0.0
            )
        );

        var gripperToTool = new RigidTransform(
            new Vector3D(
                0.04525,
                0.0,
                -0.0268
            ),
            QuaternionD.FromRollPitchYaw(
                0.0,
                Math.PI,
                0.0
            )
        );

        return flangeToGripper * gripperToTool;
    }
}
