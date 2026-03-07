package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
public class Indexer extends SubsystemBase {
    private final SparkFlex m_indexerVortex;
    private final RelativeEncoder m_indexerEncoder;
    private final SparkMax m_rollerMotor;
    private final RelativeEncoder m_rollerEncoder;
    private final SparkClosedLoopController m_indexerClosedLoop;

    public static final LoggedTunableNumber PIndexer = new LoggedTunableNumber("Indexer/kP");
    public static final LoggedTunableNumber DIndexer = new LoggedTunableNumber("Indexer/kD");

    public static final LoggedTunableNumber PRoller = new LoggedTunableNumber("Roller/kP");
    public static final LoggedTunableNumber DRoller = new LoggedTunableNumber("Roller/kD");

    public Indexer() {
        
        m_indexerVortex = new SparkFlex(MechConstants.kIndexerID, MotorType.kBrushless);
        m_indexerVortex.configure(Configs.IndexerConfigs.indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_indexerClosedLoop = m_indexerVortex.getClosedLoopController();
        m_indexerEncoder = m_indexerVortex.getEncoder();
        m_rollerMotor = new SparkMax(MechConstants.kIndexRollerID, MotorType.kBrushless);
        m_rollerEncoder = m_rollerMotor.getEncoder();
        m_rollerMotor.configure(Configs.IndexerConfigs.rollerConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
    }
    public void setIndexRPM(double rpm) {
        m_indexerClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }
    public enum IndexerState {
        RUNNING, DISABLED
    }
    IndexerState indexState = IndexerState.DISABLED;
    public void setIndexerState(IndexerState state) {
        indexState = state;
    }

    // test the indexer without a tuned pid for now
    public void setIndexRawSpeed() {
        m_indexerVortex.set(.25);
    }

    public void rollerWarmup() {
        boolean warmedUp = false;
        m_rollerMotor.set(-.25);
    }

    public Command runIndexer() {
        return this.runEnd(() -> setIndexerState(IndexerState.RUNNING),
        () -> setIndexerState(IndexerState.DISABLED));
    }




    

    public void stop() {
        m_indexerVortex.setVoltage(0);
        m_rollerMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        switch (indexState) {
            case DISABLED -> {
                stop();
            }
            case RUNNING -> {
                setIndexRawSpeed();
            }
        }
    }



   

    


}
