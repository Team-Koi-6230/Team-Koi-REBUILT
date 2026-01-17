package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class ShooterSubsystem extends SubsystemBase {
  public enum ShooterState {
    SPINUP,
    AT_TARGET,
    IDLE
  }

  private final SparkFlex m_motor;
  private final SparkFlex s_motor;
  private final SparkClosedLoopController closedLoop;
  private final RelativeEncoder encoder;

  private ShooterState state = ShooterState.IDLE;
  private double targetRPM = Double.NaN;

  public ShooterSubsystem() {
    m_motor = new SparkFlex(Constants.ShooterConstants.kMainMotorID, MotorType.kBrushless);
    encoder = m_motor.getEncoder();

    s_motor = new SparkFlex(Constants.ShooterConstants.kSecondaryMotorID, MotorType.kBrushless);


    SparkFlexConfig m_config = new SparkFlexConfig();

    m_config
      .idleMode(IdleMode.kCoast)
      .inverted(Constants.ShooterConstants.kInverted)
      .voltageCompensation(12.0);

    m_config.smartCurrentLimit(60, 80);

    m_config.encoder
      .velocityConversionFactor(1.0 / Constants.ShooterConstants.kGearRatio);

    m_config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(
        Constants.ShooterConstants.kP, 
        Constants.ShooterConstants.kI, 
        Constants.ShooterConstants.kD 
      )
    .feedForward
        .kS(Constants.ShooterConstants.kS)
        .kV(Constants.ShooterConstants.kV)
        .kA(Constants.ShooterConstants.kA);

    m_motor.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    SparkFlexConfig s_config = new SparkFlexConfig();

    s_config.follow(m_motor, !Constants.ShooterConstants.kInverted);

    s_motor.configure(s_config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    closedLoop = m_motor.getClosedLoopController();
  }

  // boilerplate command to use the subsystem
  public Command setVelocityCommand(double setpoint) {
    return runOnce(() -> {
      setTargetRPM(setpoint);
    });
  }

  public ShooterState getState() { 
    return state; 
  }

  public void setTargetRPM(double targetRPM) { 
    this.targetRPM = targetRPM; 
  }

  public double getTargetRPM() { 
    return this.targetRPM; 
  }

  public double getVelocity() { 
    return encoder.getVelocity(); 
  }

  public boolean isAtTargetVelocity() {
    if (Double.isNaN(targetRPM)) {
      return false;
    }
    return Math.abs(targetRPM - getVelocity()) < Constants.ShooterConstants.kTolerance;
  }

  public void stop() { 
    this.targetRPM = Double.NaN;
  }

  @Override
  public void periodic() {
    
     if (!Double.isNaN(targetRPM)) { 
      closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
      
      if (isAtTargetVelocity()) {
        state = ShooterState.AT_TARGET;
      } else {
        state = ShooterState.SPINUP;
      }
    } else {
      m_motor.stopMotor();
      state = ShooterState.IDLE;
    }

    SmartDashboard.putNumber("Shooter/CurrentRPM", getVelocity());
    SmartDashboard.putNumber("Shooter/TargetRPM", Double.isNaN(targetRPM) ? 0 : targetRPM);
    SmartDashboard.putString("Shooter/State", state.toString());
    SmartDashboard.putNumber("Shooter/Current", m_motor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
  }
}