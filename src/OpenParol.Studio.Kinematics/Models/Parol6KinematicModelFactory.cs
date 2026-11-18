using OpenParol.Studio.Core.Geometry;
using OpenParol.Studio.Core.Robotics;

namespace OpenParol.Studio.Kinematics.Models;

/// <summary>
/// Creates kinematic models for the PAROL6 robot.
/// </summary>
public static class Parol6KinematicModelFactory
{
    public static RobotKinematicModel Create(
        RigidTransform flangeToTool)
    {
        return new RobotKinematicModel(
            CreateJoints(),
            RigidTransform.Identity,
            CreateLastLinkToFlange(),
            flangeToTool
        );
    }

    private static RevoluteJointDefinition[] CreateJoints()
    {
        return
        [
            CreateJoint(
                name: "J1",
                translation: new Vector3D(
                    0.0,
                    0.0,
                    0.0
                ),
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    1.0
                ),
                minimum: -1.7,
                maximum: 1.7
            ),

            CreateJoint(
                name: "J2",
                translation: new Vector3D(
                    0.0234207210610365,
                    0.0,
                    0.1105
                ),
                roll: -Math.PI / 2.0,
                pitch: 0.0,
                yaw: 0.0,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    1.0
                ),
                minimum: -0.98,
                maximum: 1.0
            ),

            CreateJoint(
                name: "J3",
                translation: new Vector3D(
                    0.0,
                    -0.18,
                    0.0
                ),
                roll: Math.PI,
                pitch: 0.0,
                yaw: -Math.PI / 2.0,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    1.0
                ),
                minimum: -1.3,
                maximum: 2.0
            ),

            CreateJoint(
                name: "J4",
                translation: new Vector3D(
                    0.0435,
                    0.0,
                    0.0
                ),
                roll: Math.PI / 2.0,
                pitch: 0.0,
                yaw: -Math.PI,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    1.0
                ),
                minimum: -2.0,
                maximum: 2.0
            ),

            CreateJoint(
                name: "J5",
                translation: new Vector3D(
                    0.0,
                    0.0,
                    -0.177
                ),
                roll: -Math.PI / 2.0,
                pitch: 0.0,
                yaw: 0.0,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    1.0
                ),
                minimum: -2.1,
                maximum: 2.1
            ),

            CreateJoint(
                name: "J6",
                translation: new Vector3D(
                    0.0,
                    0.0,
                    0.0
                ),
                roll: Math.PI / 2.0,
                pitch: 0.0,
                yaw: Math.PI,
                axis: new Vector3D(
                    0.0,
                    0.0,
                    -1.0
                ),
                minimum: -3.1,
                maximum: 3.1
            )
        ];
    }

    private static RigidTransform CreateLastLinkToFlange()
    {
        return RigidTransform.FromTranslation(
            new Vector3D(
                0.0,
                0.0,
                0.0505
            )
        );
    }

    private static RevoluteJointDefinition CreateJoint(
        string name,
        Vector3D translation,
        double roll,
        double pitch,
        double yaw,
        Vector3D axis,
        double minimum,
        double maximum)
    {
        var parentToJoint = new RigidTransform(
            translation,
            QuaternionD.FromRollPitchYaw(
                roll,
                pitch,
                yaw
            )
        );

        return new RevoluteJointDefinition(
            name,
            axis,
            parentToJoint,
            new JointPositionLimits(
                minimum,
                maximum
            )
        );
    }
}
