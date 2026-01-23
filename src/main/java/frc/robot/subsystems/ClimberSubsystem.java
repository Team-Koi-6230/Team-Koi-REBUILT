// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private double targetHeight = 0.0;
    private double targetVelocity = 0.0; 
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoop;
    private final ElevatorFeedforward feedforward;
  public ClimberSubsystem() {
    motor1 = new SparkMax(ClimberConstants.MOTOR1_CAN_ID, MotorType.kBrushless);
    motor2 = new SparkMax(ClimberConstants.MOTOR2_CAN_ID, MotorType.kBrushless);
    encoder = motor1.getEncoder();
    closedLoop = motor1.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(ClimberConstants.METERS_PER_ROTATION)
        .velocityConversionFactor(ClimberConstants.METERS_PER_ROTATION / 60.0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.p(ClimberConstants.kP);
    config.closedLoop.i(ClimberConstants.kI);
    config.closedLoop.d(ClimberConstants.kD);
    config.closedLoop.feedForward.kV(ClimberConstants.kF);
    motor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerConfig.follow(motor1.getDeviceId()); 
    followerConfig.inverted(false);
    motor2.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    feedforward = new ElevatorFeedforward(
        ClimberConstants.kS,
        ClimberConstants.kV,
        ClimberConstants.kG,
        ClimberConstants.kA
    );
  }
  public void moveUp() 
  {
    targetVelocity = 0.6;  
  }
  public void moveDown() 
  {
    targetVelocity = -0.6;
  }
  public void stop() 
  {
    targetVelocity = 0.0;
  }
  public double getHeight() 
  {
    return encoder.getPosition();
  }

  public double getVelocity() 
  {
    return encoder.getVelocity();
  }
  public void setPosition(double targetHeight) 
  {
    this.targetHeight = targetHeight;
  }
  @Override
  public void periodic() {
    double ffVolts = feedforward.calculate(targetVelocity);
        closedLoop.setSetpoint(
            targetVelocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
  }
}