package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretAzimuth;
import frc.robot.subsystems.Turret.TurretShooter;
import edu.wpi.first.math.geometry.Twist2d;

import frc.robot.subsystems.Turret.Indexer;
import frc.robot.subsystems.Turret.Indexer.IndexerState;
import frc.robot.utils.FieldConstants;

public class TurretVisionAim extends Command {
    private final SwerveSubsystem driveSubsystem;
    private final TurretAzimuth m_turretAngle;
    private final TurretShooter m_turretShooter;
    private final Indexer m_indexer;

    private Translation2d targetLocation;

    public TurretVisionAim(SwerveSubsystem drive, TurretAzimuth turret, TurretShooter turretshoot, Indexer indexer) {
        targetLocation = new Translation2d(0, 0);
        driveSubsystem = drive;
        m_turretAngle = turret;
        m_turretShooter = turretshoot;
        m_indexer = indexer;

        addRequirements(m_turretAngle, m_turretShooter, m_indexer);
        setName("aimturretandshoot");

    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                targetLocation = FieldConstants.HUB_BLUE;
            } else {
                targetLocation = FieldConstants.HUB_RED;
            }
        }
    }

    @Override
    public void execute() {

        double latencySeconds = 0.0; // mechanical delay
        Pose2d currentPose = driveSubsystem.getPose();
        var speeds = driveSubsystem.getRobotVelocity();

        Twist2d twist = new Twist2d(
                speeds.vxMetersPerSecond * latencySeconds,
                speeds.vyMetersPerSecond * latencySeconds,
                speeds.omegaRadiansPerSecond * latencySeconds);

        // apply twist to get where the robot WILL be when the ball actually fires
        Pose2d predictedPose = currentPose.exp(twist);
        Translation2d robotLocation = predictedPose.getTranslation();

        Translation2d fieldRelativeVelocity = new Translation2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond).rotateBy(currentPose.getRotation());

        double virtualDistance = robotLocation.getDistance(targetLocation);
        double timeOfFlight = 0.0;
        Translation2d virtualTarget = targetLocation;

        double vx = fieldRelativeVelocity.getX();
        double vy = fieldRelativeVelocity.getY();
        for (int i = 0; i < 4; i++) {
            timeOfFlight = m_turretShooter.getEstimatedTimeOfFlight(virtualDistance);
            double vtx = targetLocation.getX() - vx * timeOfFlight;
            double vty = targetLocation.getY() - vy * timeOfFlight;
            virtualDistance = Math.hypot(
                    robotLocation.getX() - vtx,
                    robotLocation.getY() - vty);
            virtualTarget = new Translation2d(vtx, vty);
        }

        double dx = virtualTarget.getX() - robotLocation.getX();
        double dy = virtualTarget.getY() - robotLocation.getY();

        // field-centric angle to the virtual target
        Rotation2d fieldRelativeAngleToTarget = new Rotation2d(Math.atan2(dy, dx));

        // turret setpoint is field-centric angle subtracted by the robot's predicted
        // rotation
        Rotation2d virtualTurretSetpoint = fieldRelativeAngleToTarget.minus(predictedPose.getRotation());

        // feed final converged distance to shooter and angle to turret

        m_turretShooter.prepareToShoot(virtualDistance);
        m_turretAngle.setTargetAngle(virtualTurretSetpoint.getDegrees());

        boolean isShooterReady = m_turretShooter.isAtTargetRPM();
        boolean isTurretReady = m_turretAngle.isAtTargetAngle();

        if (isShooterReady && isTurretReady) {

            m_indexer.setIndexerState(IndexerState.RUNNING);
        } else {
            // keep warming up the rollers but don't feed the ball into the flywheel yet
            m_indexer.setIndexerState(IndexerState.DISABLED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // when you let go of the button stop everything.
        m_indexer.setIndexerState(IndexerState.DISABLED);

        m_turretShooter.stop();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}