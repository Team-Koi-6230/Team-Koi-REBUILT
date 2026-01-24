// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public enum ClimberState {
    MOVING,
    AT_TARGET
  }

  private double targetHeight = 0.0;
  private ClimberState state = ClimberState.MOVING;
  private final SparkMax m_motor;
  private final SparkMax s_motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoop;

  public ClimberSubsystem() {
    m_motor = new SparkMax(Constants.ClimberConstants.kMainMotorID, MotorType.kBrushless);
    s_motor = new SparkMax(Constants.ClimberConstants.kSecondaryMotorID, MotorType.kBrushless);
    encoder = m_motor.getEncoder();
    closedLoop = m_motor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.ClimberConstants.kP,
            Constants.ClimberConstants.kI,
            Constants.ClimberConstants.kD)

                .feedForward
        .kS(Constants.ClimberConstants.kS)
        .kG(Constants.ClimberConstants.kG)
        .kV(Constants.ClimberConstants.kV)
        .kA(Constants.ClimberConstants.kA);

    config.idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(Constants.ClimberConstants.kMetersPerRotation)
        .velocityConversionFactor(Constants.ClimberConstants.kMetersPerRotation / 60.0);

    m_motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    followerConfig.follow(m_motor, true);
    s_motor.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

  }

  public void stop() {
    m_motor.stopMotor();
  }

  public ClimberState getState() {
    return state;
  }

  public double getHeight() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setPosition(double targetHeight) {
    this.targetHeight = targetHeight;
  }

  @Override
  public void periodic() {
    closedLoop.setSetpoint(targetHeight, ControlType.kPosition);
    if (Math.abs(targetHeight - getHeight()) < Constants.ClimberConstants.kTolerance) {
      state = ClimberState.AT_TARGET;
    } else {
      state = ClimberState.MOVING;
    }
  }
}