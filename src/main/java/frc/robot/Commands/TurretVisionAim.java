package frc.robot.Commands;

import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.Vision;
import frc.robot.utils.LimelightHelpers;
import swervelib.math.SwerveMath;
import frc.robot.subsystems.Turret.Turret;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class TurretVisionAim extends Command {

    private Turret m_turret = new Turret();

    private static double m_angularVelocity;
    private static double m_targetError;
    
    


    public TurretVisionAim() {
        
        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

