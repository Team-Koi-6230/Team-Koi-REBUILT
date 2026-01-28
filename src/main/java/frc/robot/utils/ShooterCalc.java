package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.HubRelativeVelocity;

public class ShooterCalc {
    private static final InterpolatingDoubleTreeMap shotHoodAngleMap = Constants.ShooterConstants.kShotHoodAngleMap;
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = Constants.ShooterConstants.kShotFlywheelSpeedMap;
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = Constants.ShooterConstants.kTimeOfFlightMap;

    private static final LinearFilter hoodFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    public record ShootingParameters(
            boolean isValid,
            double hoodAngle,
            double flywheelSpeed,
            Pose2d target) {
    }

public static ShootingParameters getParameters() {
    // Get current robot pose
    SwerveSubsystem swerveDrive = Superstructure.getInstance().getDrivebase();
    Pose2d robotPose = swerveDrive.getPose();

    // Base distance to hub
    Translation2d hubPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    double distance = robotPose.getTranslation().getDistance(hubPos);

    // Check if in valid shooting range
    if (distance < 0 || distance > Constants.ShooterConstants.kMaxShootingDist) {
        return new ShootingParameters(false, 0.0, 0.0, new Pose2d());
    }

    // Lead the shot based on robot velocity
    HubRelativeVelocity hubVel = swerveDrive.getHubRelativeVelocity();
    double timeOfFlight = timeOfFlightMap.get(distance);

    // Compute lead offsets
    double leadRadialOffset = hubVel.radialSpeed() * timeOfFlight;
    double leadStrafeOffset = hubVel.strafeSpeed() * timeOfFlight;

    // Create lead-corrected target
    Translation2d leadTarget = hubPos.minus(new Translation2d(leadRadialOffset, leadStrafeOffset));

    // Distance to lead target
    double leadDistance = robotPose.getTranslation().getDistance(leadTarget);

    // Get hood angle and flywheel speed from lookup tables
    double hoodAngle = shotHoodAngleMap.get(leadDistance);
    double baseFlywheelSpeed = shotFlywheelSpeedMap.get(leadDistance);

    // Apply radial speed compensation to flywheel RPM
    double radialComp = Constants.ShooterConstants.kRadialRPMComp * hubVel.radialSpeed();
    double finalFlywheelSpeed = baseFlywheelSpeed + radialComp;

    // Filter hood angle for smooth servo movement
    hoodAngle = hoodFilter.calculate(hoodAngle);

    return new ShootingParameters(
            true,
            hoodAngle,
            finalFlywheelSpeed,
            new Pose2d(leadTarget.getX(), leadTarget.getY(), new Rotation2d())
    );
}

    public static void resetHoodFilter() {
        hoodFilter.reset();
    }
}
