package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4; // Max speed = 5.0
    public static final double kMaxAngularSpeed = 2.25 * Math.PI; // radians per second  max is 4 so far?
    public static final double kWheelDiameter = 0.0762;
    public static final double kDirectionSlewRate = 2.3; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRobotMOI = 1.6; 
    public static final double kRobotMass = 49.8952; 
    public static final double kWheelCOF = 1.6; 
    // ==Chassis configuration==
    public static final double kTrackWidth = Units.inchesToMeters(29);
    // ==Distance between centers of right and left wheels on robot==
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    

    public static final boolean kGyroReversed = false;
    // -- ADJUST ALL OF THIS ONCE BOT IS BUILT --

    // -- limelight constants for the turret --

    // turret location relative to the center of robot
    public static final double TURRET_CENTER_X_OFFSET = 0.0; // forward of robot center
    public static final double TURRET_CENTER_Y_OFFSET = 0.0; // left/right of robot center

    //camera location relative to the center of the turret
    public static final double CAMERA_RADIUS_FROM_TURRET = 0.20; // distance from center of turret
    public static final double CAMERA_Z_HEIGHT = 0.55; // Height of camera lens from the floor
    
    public static final double CAMERA_PITCH = 20.0; // Static angle the camera tilts up/down
    public static final double CAMERA_ROLL = 0.0;

    // chassis camera location relative to robots center

    public static final double CHASSIS_CAMERA_FORWARD_OFFSET = 0.20; // distance from center of robot
    public static final double CHASSIS_CAMERA_Z_HEIGHT = 0.55; // Height of camera lens from the floor
    public static final double CHASSIS_CAMERA_YAW = 180; // angle camera is facing relative to front
    public static final double CHASSIS_CAMERA_PITCH = 20.0; // Static angle the camera tilts up/down
    public static final double CHASSIS_CAMERA_ROLL = 0.0; 

}
