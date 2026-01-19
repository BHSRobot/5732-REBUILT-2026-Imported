package frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Turret extends SubsystemBase {

    private final SparkFlex m_mainVortex;
    private final SparkFlex m_altVortex;
    private double m_currentRotation;

    public Turret() {
        m_mainVortex = new SparkFlex(0, MotorType.kBrushless);
        m_altVortex = new SparkFlex(0, MotorType.kBrushless);
    }


    @Override
    public void periodic() {
        
    }

    public void getCurrentAngle() {
        
    }

    
}
