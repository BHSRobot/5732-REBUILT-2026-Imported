package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.Configs;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    private IntakeState iState = IntakeState.DISABLED;

    
    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); 
        //Logger.processInputs("Intake", (LoggableInputs) inputs); // If using AdvantageKit

        switch (iState) {
            case DISABLED -> {
                io.setIntakeVoltage(0.0);
                io.setExtended(false);
            }
            case EJECTING -> {
                io.setIntakeRPM(-3600);
                io.setExtended(true);
            }
            case INTAKING -> {
                io.setIntakeRPM(3600);
                io.setExtended(true);
            }
        }
    }

    public void setIntakeState(IntakeState state) {
        iState = state;
    }
    
    public enum IntakeState { INTAKING, EJECTING, DISABLED }

    public Command intakeCommand() {
        return this.runEnd(() -> setIntakeState(IntakeState.INTAKING), 
                           () -> setIntakeState(IntakeState.DISABLED));
    }

    public Command ejectCommand() {
        return this.runEnd(() -> setIntakeState(IntakeState.EJECTING), 
                           () -> setIntakeState(IntakeState.DISABLED));
    }
}
