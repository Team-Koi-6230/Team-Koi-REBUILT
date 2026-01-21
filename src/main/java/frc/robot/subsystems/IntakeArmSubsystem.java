package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
    private final DutyCycleEncoder m_absoluteEncoder;
    private final SparkMax m_motor;
    private SparkClosedLoopController m_controller;
    private final RelativeEncoder m_relativeEncoder;

    private double targetAngle;
     
     public enum IntakeArmState{
        IDLE,
        OPEN,
        CLOSED,
        MOVING
     }
     public IntakeArmState state = IntakeArmState.IDLE;

    public IntakeArmSubsystem() 
    {
        // set absolute encoder
        m_absoluteEncoder = new DutyCycleEncoder(
        IntakeArmConstants.kAbsoluteEncoderID, 
        IntakeArmConstants.kAbsoluteEncoderRange, 
        IntakeArmConstants.kAbsoluteEncoderOffset
        );
         
        // set motor
        m_motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
        targetAngle = 0;

        // set config pid & ff
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            IntakeArmConstants.kP,
            IntakeArmConstants.kI,
            IntakeArmConstants.kD
        )
        .feedForward
        .kS(IntakeArmConstants.kS)
        .kV(IntakeArmConstants.kV)
        .kA(IntakeArmConstants.kA)
        .kCos(IntakeArmConstants.kG)
        .kCosRatio(IntakeArmConstants.kCosRatio);
        


        // set motor config
        m_motor.configure(config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

        // set relative encoder
        m_relativeEncoder = m_motor.getEncoder();

        // set motor contoller
        m_controller = m_motor.getClosedLoopController();

       // set the relative position to the duty position
        m_relativeEncoder.setPosition(m_absoluteEncoder.get());
    }
      
    public Command IntakeArmCommand(double angle) {

        return runOnce(() -> {
            setAngle(angle);
        });
    }
    
    //set target angle 
    public void setAngle(double angle){
        this.targetAngle = angle;
        state = IntakeArmState.MOVING;
    }

    public double getAngle() 
    {
        return m_relativeEncoder.getPosition();
    }
     
    private boolean isOpen() 
    {
        return Math.abs(IntakeArmConstants.kOpenAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    private boolean isClosed() 
    {
        return Math.abs(IntakeArmConstants.kClosedAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }
        

     @Override
    public void periodic() {
        m_controller.setSetpoint(this.targetAngle, ControlType.kPosition);
        
        if (Double.isNaN(this.targetAngle)) 
        {
            state = IntakeArmState.IDLE;
        }
        else if (isOpen()) 
        {
            state = IntakeArmState.OPEN;            
        }
        else if (isClosed())
        {
            state = IntakeArmState.CLOSED;
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}