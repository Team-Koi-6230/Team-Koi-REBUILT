package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.ScoreCommand.ShooterPoint;
import frc.robot.utils.RumbleSubsystem.Priority;
import frc.robot.utils.RumblePack;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.3;

    // Rumble constants
    public static final RumblePack kGameShiftRumble = new RumblePack(0.7, 0.5, Priority.HIGH);

    // Game data management
    public static final int kTeleopInterval = 25;
    public static final int kEndGameTime = 30;
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kFieldApril = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    private static final Pose2d kBlueHub = new Pose2d(0, 0, new Rotation2d());
    private static final Pose2d kRedHub = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d getHubPose() { 
      return DriverStation.getAlliance()
        .orElse(Alliance.Blue) == Alliance.Red
            ? kRedHub
            : kBlueHub;
    }
  }

  public static class VisionConstants {
    public static final String kLimelightName = "";
    public static final double kAimTolerance = 0.2;
  }

  public static class SwerveDriveConstants {
    public static final double kMaxSpeed = Units.feetToMeters(20);
    public static final double kVisionPeriod = 0.1; // 10Hz 
    public static final double kTargetErrorTolerance = Math.toRadians(3);
    public static final double kPr = 0.0, kIr = 0.0, kDr = 0.0;
  }

  public static class ShooterConstants {
    public static final int kMainMotorID = 0;
    public static final int kSecondaryMotorID = 0;
    
    public static final double kGearRatio = 1.0;
    public static final double kTolerance = 20.0;

    public static final boolean kInverted = false;
    
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    // fake data for now
    public static final ShooterPoint[] kShooterLUT = {
      new ShooterPoint(2.0, 3100),
      new ShooterPoint(2.5, 3350),
      new ShooterPoint(3.0, 3600),
      new ShooterPoint(3.5, 3900),
      new ShooterPoint(4.0, 4250)
  };

  public static final double kMaxShootingDist = 4.0;
}

public static class IntakeArmConstants{
  // WRONG VALUES! set the IDs and values later
  public static final int kAbsoluteEncoderID = 0;
  public static final int kMotorID = 0;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kFF = 0;
    
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kG = 0;
  public static final double kCosRatio = 1;

  public static final int ABS_ENCODER_CHANNEL = 0;
  public static final int ABS_ENCODER_RANGE = 360;
  public static final int ABS_ENCODER_OFFSET = 0;

  public static final double kTolerance = 0.5;

  public static final int OPEN_ANGLE = 67;
  public static final int CLOSED_ANGLE = 0;
}
}
