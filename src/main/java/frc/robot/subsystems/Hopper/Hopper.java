package frc.robot.subsystems.Hopper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.DigitalInput;


public class Hopper extends SubsystemBase {
    private final DigitalInput breakBeam;
    private final SparkFlex m_mainVortex;
    private final AbsoluteEncoder m_hoppEncoder;
    
    private double m_currentHeight;

    public Hopper() {
        m_mainVortex = new SparkFlex(Constants.MechConstants.kHoppLenID, MotorType.kBrushless);
        m_hoppEncoder = m_mainVortex.getAbsoluteEncoder();
        breakBeam = new DigitalInput(Constants.MechConstants.kBreakBeamChannel);
    }


    @Override
    public void periodic() {
        m_currentHeight = m_hoppEncoder.getPosition() * Constants.MechConstants.kHopperLenConversionFactor;
    }

    // returns if its fully extended
    public boolean isExtended() {
        return m_currentHeight == Constants.MechConstants.kHoppFullExtPos;
    }

    public boolean isFull() {
        // when the beam is broken it will return false
        return !breakBeam.get();
    }


    
}
