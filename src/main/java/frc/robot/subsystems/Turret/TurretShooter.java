package frc.robot.subsystems.Turret;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import com.revrobotics.PersistMode;
public class TurretShooter extends SubsystemBase {
    private static double m_currentVelocity;
    private static double m_targetVelocity;
    private double m_currentHoodAngle;
    private double m_targetHoodAngle;

    private final SparkFlex m_shooterFlex; 
    private final SparkClosedLoopController m_shooterClosedLoop;
    private final SparkMax m_hoodMotor;
    private final SparkClosedLoopController m_turretHoodClosedLoop;
    public final InterpolatingDoubleTreeMap m_shooterMap;
    
    public static final LoggedTunableNumber m_PTurretHood = new LoggedTunableNumber("TurretHood/kP");
    public static final LoggedTunableNumber m_DTurretHood = new LoggedTunableNumber("TurretHood/kD");

    public TurretShooter() {
        m_shooterMap = new InterpolatingDoubleTreeMap();
        m_targetVelocity = 0.0;
        m_shooterFlex = new SparkFlex(Constants.MechConstants.kTurrShootID, MotorType.kBrushless);
        m_shooterFlex.configure(Configs.TurretConfigs.shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterClosedLoop = m_shooterFlex.getClosedLoopController();
        m_hoodMotor = new SparkMax(MechConstants.kTurrHoodID, MotorType.kBrushless);
        m_hoodMotor.configure(Configs.TurretConfigs.azimuthConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turretHoodClosedLoop = m_hoodMotor.getClosedLoopController();

    }
    
    
       
    


    @Override
    public void periodic() {
        Logger.recordOutput("TurretHood/CurrentAngle", m_currentHoodAngle);
        Logger.recordOutput("TurretHood/targetAngle", m_targetHoodAngle);
        m_PTurretHood.initDefault(TurretConstants.kPTurretHood);
        m_DTurretHood.initDefault(TurretConstants.kDTurretHood);
        if (Constants.tuningMode) {
            if (m_PTurretHood.hasChanged(hashCode()) || m_DTurretHood.hasChanged(hashCode()))  {
                SparkMaxConfig updateConfig = new SparkMaxConfig();
                updateConfig.closedLoop.pid(m_PTurretHood.get(), 0.0, m_DTurretHood.get());
                m_hoodMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }
    }

    public double getCurrentHoodAngle() {
        return m_currentHoodAngle;
    }

    public double getFlywheelRPM() {
        return m_currentVelocity;
    }

    public void setTargetHoodAngle(double angle) {
        m_targetHoodAngle = angle;
        m_turretHoodClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }



    

    public void setFlywheelRPM(double rpm) {
        m_targetVelocity = rpm;
        m_shooterClosedLoop.setSetpoint(m_targetVelocity, ControlType.kMAXMotionVelocityControl);
    }

}
