package frc.robot;
import java.util.Map;
import edu.wpi.first.math.util.Units;

import frc.robot.commands.ScoreCommand.ShooterPoint;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.3;
  }

  public static class VisionConstants {
    public static final String kLimelightName = "";

    public static final Map<Integer, Boolean> scoringTags = Map.of(
      // unintellegent pre-season data
     1,true,
     5, true
    );
    public static final double kAmbiguityTolerance = 0.7;
  }

  public static class SwerveDriveConstants {
    public static final double kMaxSpeed = Units.feetToMeters(20);
    public static final double kVisionPeriod = 0.1; // 10Hz
    public static final double kTargetErrorTolerance = Math.toRadians(3);
    public static final double kRotationP = 0.25;
  }

  public static class ShooterConstants {
    public static final int kMainMotorID = 0;
    
    public static final double kGearRatio = 1.0;
    public static final double kTolerance = 50.0;
    
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    public static final double kS = 0.1;
    public static final double kV = 0.12;
    public static final double kA = 0.01;

    // fake data for now
    public static final ShooterPoint[] kShooterLUT = {
      new ShooterPoint(2.0, 38, 3100),
      new ShooterPoint(2.5, 40, 3350),
      new ShooterPoint(3.0, 42, 3600),
      new ShooterPoint(3.5, 44, 3900),
      new ShooterPoint(4.0, 46, 4250),
  };
}
}
