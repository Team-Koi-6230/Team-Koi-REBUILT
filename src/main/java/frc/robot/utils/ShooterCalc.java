package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Superstructure;

public class ShooterCalc {
    private static final InterpolatingDoubleTreeMap shotHoodAngleMap = Constants.ShooterConstants.kShotHoodAngleMap;
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = Constants.ShooterConstants.kShotFlywheelSpeedMap;
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = Constants.ShooterConstants.kTimeOfFlightMap;

    private static final LinearFilter hoodFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
    private static final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private static final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public record ShootingParameters(
            boolean isValid,
            double hoodAngle,
            double flywheelSpeed,
            Pose2d target) {
    }

    public static ShootingParameters getParameters() {
        var swerve = Superstructure.getInstance().getDrivebase();
        Pose2d robotPose = swerve.getPose();
        Translation2d hubPos = FieldConstants.Hub.innerCenterPoint.toTranslation2d();

        // 1. Get Distance
        double distance = robotPose.getTranslation().getDistance(hubPos);

        // Guard Clause
        if (distance < 0 || distance > Constants.ShooterConstants.kMaxShootingDist) {
            return new ShootingParameters(false, 0.0, 0.0, new Pose2d());
        }

        double timeOfFlight = timeOfFlightMap.get(distance);

        // 2. Get Raw Velocity
        ChassisSpeeds fieldSpeeds = swerve.getFieldVelocity();

        // 3. FILTER THE VELOCITY (The Fix)
        // We smooth the X and Y components individually to remove sensor noise
        double smoothVx = xFilter.calculate(fieldSpeeds.vxMetersPerSecond);
        double smoothVy = yFilter.calculate(fieldSpeeds.vyMetersPerSecond);

        Translation2d smoothedVelocity = new Translation2d(smoothVx, smoothVy);

        // 4. Calculate Lead with Smoothed Data
        // Virtual Target = Hub - (Velocity * Time)
        Translation2d leadOffset = smoothedVelocity.times(timeOfFlight);
        Translation2d virtualTarget = hubPos.minus(leadOffset);

        // 5. Final Calculations
        double leadDistance = robotPose.getTranslation().getDistance(virtualTarget);

        double hoodAngle = hoodFilter.calculate(shotHoodAngleMap.get(leadDistance));
        double flywheelSpeed = shotFlywheelSpeedMap.get(leadDistance);

        return new ShootingParameters(
                true,
                hoodAngle,
                flywheelSpeed,
                new Pose2d(virtualTarget, new Rotation2d()));
    }

    public static void resetHoodFilter() {
        hoodFilter.reset();
    }
}
