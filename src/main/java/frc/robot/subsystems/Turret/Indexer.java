package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants.MechConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
public class Indexer extends SubsystemBase {
    private final SparkFlex m_indexerVortex;
    private final SparkClosedLoopController m_indexerClosedLoop;

    public Indexer() {
        m_indexerVortex = new SparkFlex(MechConstants.kIndexerID, MotorType.kBrushless);
        m_indexerVortex.configure(Configs.TurretConfigs.indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_indexerClosedLoop = m_indexerVortex.getClosedLoopController();
    }
    public void setIndexRPM(double rpm) {
        m_indexerClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    public void stop() {
        m_indexerVortex.setVoltage(0);
    }

    @Override
    public void periodic() {
        
    }

    


}
