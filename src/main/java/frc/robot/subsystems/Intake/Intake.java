package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.Configs;

public class Intake extends SubsystemBase {
    private IntakeState iState = IntakeState.DISABLED;
    private final SparkFlex intakeMotor;
    private double motorSpeed;
    public Intake() {
        intakeMotor = new SparkFlex(Constants.MechConstants.kIntakeID, MotorType.kBrushless);
        intakeMotor.configure(Configs.IntakeConfigs.intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        switch (iState) {
            case DISABLED:
                intakeMotor.set(0);
                break;
            case EJECTING:
                intakeMotor.set(-1);
                break;
            case INTAKING:
                intakeMotor.set(1);
                break;
        }
            
    }

    public void setIntakeState(IntakeState state) {
            iState = state;
        
    }
    // for debugging intake
    public IntakeState getIntakeState() {
        return iState;
    }
    
    
    public enum IntakeState {
        INTAKING,
        EJECTING,
        DISABLED
    }
}
