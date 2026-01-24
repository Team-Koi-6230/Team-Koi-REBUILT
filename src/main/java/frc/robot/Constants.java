package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.ScoreCommand.ShooterPoint;
import frc.robot.utils.RumbleSubsystem.Priority;
import frc.robot.utils.RumblePack;

public final class Constants {
  public static class OperatorConstants {

    public final class ClimberConstants {
      public static double kS = 0.0;
      public static double kF = 0.0;
      public static double kG = 0.0;
      public static double kV = 0.0;
      public static double kA = 0.0;
      public static double kP = 0.0;
      public static double kI = 0.0;
      public static double kD = 0.0;
      public static double tolerance = 0.0;

      public static int MOTOR1_CAN_ID = 0;
      public static int MOTOR2_CAN_ID = 0;

      public static double METERS_PER_ROTATION = 0.0;

    }

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
    // We'll wait for 6238 positions for that
    private static final Pose2d kBlueHub = new Pose2d();
    private static final Pose2d kRedHub = new Pose2d();

    public static Pose2d getHubPose() {
      return DriverStation.getAlliance()
          .orElse(Alliance.Blue) == Alliance.Red
              ? kRedHub
              : kBlueHub;
    }
  }

  public static class VisionConstants {
    public static final String kLimelightName = "limelight-front";
    public static final double kAimTolerance = 0.2;
  }

  public static class SwerveDriveConstants {
    public static final double kMaxSpeed = 5.36448;
    public static final double kMaxAngularVelocity = Math.PI;
    public static final double kAimingSpeedModifier = 0.5;
    public static final double kVisionPeriod = 0.1; // 10Hz
    public static final double kTargetErrorTolerance = Math.toRadians(3);
    public static final double kPr = 0.0, kIr = 0.0, kDr = 0.0;
  }

  public static class ShooterConstants {
    public static final int kMainMotorID = 15;
    public static final int kSecondaryMotorID = 16;

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

    public static final RumblePack kRumbleScoreReady = new RumblePack(0.3, 0.2, Priority.MEDIUM);
  }
  
  public static class HoodConstants {
    public static final int kServoIdYehodi = 0;
    public static final int kServoIdGoy = 0;
    public static final double kMinDeg = 0;
    public static final double kMaxDeg = 0;
    public static final int kServoMin = 0;
    public static final int kServoMax = 0;
    public static final int kStartingPos = 0;
    public static final double kServoDelay = 0.1;
  }

  public static class IntakeArmConstants {
    public static final int kAbsoluteEncoderID = 0;
    public static final int kMotorID = 17;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double kCosRatio = 1;

    public static final int kAbsoluteEncoderRange = 360;
    public static final int kAbsoluteEncoderOffset = 0;

    public static final double kTolerance = 0.5;

    public static final int kOpenAngle = 67;
    public static final int kClosedAngle = 0;
  }

  public static class IntakeRollerConstants {
    public static final int kMotorID = 18;

    public static final double kIntakePower = 0.5;

    public static final RumblePack kIntakeReadyRumble = new RumblePack(0.3, 0.2, Priority.LOW);
  }

  public static class FeederConstants {
    public static final int kMotorID = 19;

    public static final double kGrabPower = 0.2;
  }

  public final class ClimberConstants {
    public static double kS = 0.0;
    public static double kF = 0.0;
    public static double kG = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kTolerance = 0.5;

    public static int kMainMotorID = 20;
    public static int kSecondaryMotorID = 21;

    public static double kMetersPerRotation = 0.0;

  }
}
