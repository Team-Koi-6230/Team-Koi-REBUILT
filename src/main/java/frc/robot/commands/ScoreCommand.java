package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SwerveSubsystem.SwerveState;

public class ScoreCommand extends Command {
    public record ShooterPoint(
        double distanceMeters,
        double angleDeg,
        double rpm
    ) {}

    private final ShooterSubsystem shooterSubsystem;
    private final Vision vision;
    private final SwerveSubsystem drivebase;
    private final ArmSubsystem armSubsystem;


    public ScoreCommand(ShooterSubsystem shooterSubsystem, Vision vision, SwerveSubsystem drivebase, ArmSubsystem armSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.vision = vision;
        this.drivebase = drivebase;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        drivebase.AimAtScoringAprilla();
        if (drivebase.getState() != SwerveState.VISION_LOCKED) return;
        var f = vision.getScoringTag();
        if (!f.isPresent()) return;
        double distance = f.get().txnc;

        ShooterPoint target = interpolate(distance);

        shooterSubsystem.setTargetRPM(target.rpm);
        armSubsystem.setTargetAngle(target.angleDeg);
        if (shooterSubsystem.getState() != ShooterState.AT_TARGET) return;
        if (armSubsystem.getState() != ArmState.AT_TARGET) return;

        // release the ball or smth I guess
    }

    public static ShooterPoint interpolate(double distance) {
    final ShooterPoint[] shooterLUT = Constants.ShooterConstants.kShooterLUT;
    for (int i = 0; i < shooterLUT.length- 1; i++) {
        ShooterPoint p1 = shooterLUT[i];
        ShooterPoint p2 = shooterLUT[i + 1];

        if (distance >= p1.distanceMeters()
         && distance <= p2.distanceMeters()) {

            double t = (distance - p1.distanceMeters())
                     / (p2.distanceMeters() - p1.distanceMeters());

            return new ShooterPoint(
                distance,
                lerp(p1.angleDeg(), p2.angleDeg(), t),
                lerp(p1.rpm(),      p2.rpm(),      t)
            );
        }
    }

    // Clamp to ends
    if (distance < shooterLUT[0].distanceMeters())
        return shooterLUT[0];
    return shooterLUT[shooterLUT.length - 1];
}

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
