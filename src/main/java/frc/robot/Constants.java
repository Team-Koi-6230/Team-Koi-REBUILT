package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.commands.ScoreCommand.ShooterPoint;
import frc.robot.utils.RumbleSubsystem.Priority;
import frc.robot.utils.RumblePack;

public final class Constants {
  public static boolean disableHAL = false;

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

  public static class VisionConstants {
    public static final String kLimelightName = "limelight-front";
    public static final double kAimTolerance = 0.2;
  }

  public static class SwerveDriveConstants {
    public static final double kMaxSpeed = 5.36448;
    public static final double kMaxAngularVelocity = Math.PI;
    public static final double kAimingSpeedModifier = 0.5;
    public static final double kMaxStrafe = 0.5; // max strafe speed while aiming
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

    public static final double kS = 0.2; // static friction
    public static final double kV = 0.1665; // volts per RPS
    public static final double kA = 0.002; // acceleration factor

    public static final double kP = 0.003; // PID for fine-tune
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // fake data for now
    public static final ShooterPoint[] kShooterLUT = {
        new ShooterPoint(2.0, 3100, 45),
        new ShooterPoint(2.5, 3350, 46),
        new ShooterPoint(3.0, 3600, 47),
        new ShooterPoint(3.5, 3900, 50),
        new ShooterPoint(4.0, 4250, 67)
    };

    public static final InterpolatingDoubleTreeMap kRpmMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kHoodMap = new InterpolatingDoubleTreeMap();

    static {
      for (ShooterPoint p : Constants.ShooterConstants.kShooterLUT) {
        kRpmMap.put(p.distanceMeters(), p.rpm());
        kHoodMap.put(p.distanceMeters(), p.hoodAngle());
      }
    }

    public static final double kMaxShootingDist = 4.0;
    public static final double kRadialRPMComp = 150; // what rpm we need to compensate when driving backwards from the
                                                     // hub @ max accel

    public static final RumblePack kRumbleScoreReady = new RumblePack(0.3, 0.2, Priority.MEDIUM);
  }

  public static class HoodConstants {
    public static final int kServoRightID = 0;
    public static final int kServoLeftID = 1;
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
    public static double kS_ground = 0.0;
    public static double kG_ground = 0.0;
    public static double kV_ground = 0.0;
    public static double kA_ground = 0.0;

    public static double kP_ground = 0.0;
    public static double kI_ground = 0.0;
    public static double kD_ground = 0.0;

    public static double kS_hang = 0.0;
    public static double kG_hang = 0.0;
    public static double kV_hang = 0.0;
    public static double kA_hang = 0.0;

    public static double kP_hang = 0.0;
    public static double kI_hang = 0.0;
    public static double kD_hang = 0.0;

    public static double kTolerance = 0.5;

    public static int kMainMotorID = 20;
    public static int kSecondaryMotorID = 21;
    public static int kDutyCycleChannel = 1;

    public static int kDutyCycleOffset = 0;

    public static double kMetersPerRotation = 0.0;

  }
}
