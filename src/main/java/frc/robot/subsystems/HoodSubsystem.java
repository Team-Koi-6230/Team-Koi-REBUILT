package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    public enum HoodState {
        AT_TARGET,
        MOVING
    }

    private final Servo servoRight, servoLeft;
    private double targetAngle = Double.NaN; // current goal
    private HoodState state = HoodState.AT_TARGET;

    private WantedState currentWantedState;

    public HoodSubsystem() {
        servoRight = new Servo(Constants.HoodConstants.kServoRightID);
        servoLeft = new Servo(Constants.HoodConstants.kServoLeftID);

        // Start at initial position
        setAngle(Constants.HoodConstants.kStartingPos);
        targetAngle = Constants.HoodConstants.kStartingPos;
        state = HoodState.AT_TARGET;
    }

    
    public void setAngle(double degrees) {
        degrees = MathUtil.clamp(degrees, Constants.HoodConstants.kMinDeg, Constants.HoodConstants.kMaxDeg);

        double normalized = (degrees - Constants.HoodConstants.kMinDeg)
                / (Constants.HoodConstants.kMaxDeg - Constants.HoodConstants.kMinDeg);

        double leftPwm = Constants.HoodConstants.kServoMin
                + normalized * (Constants.HoodConstants.kServoMax - Constants.HoodConstants.kServoMin);

        double rightPwm = Constants.HoodConstants.kServoMin
                + (1.0 - normalized) * (Constants.HoodConstants.kServoMax - Constants.HoodConstants.kServoMin);

        servoRight.set(leftPwm);
        servoLeft.set(rightPwm);

        // Update target and state
        targetAngle = degrees;
        state = HoodState.MOVING;

        Timer.delay(Constants.HoodConstants.kServoDelay);

        state = HoodState.AT_TARGET;
    }

    /** Returns current hood state (AT_TARGET or MOVING) */
    public HoodState getState() {
        return state;
    }

    /** Command wrapper to set hood angle once */
    public Command setHoodAngleCommand(double angle) {
        return runOnce(() -> setAngle(angle));
    }

    /** Returns the current target angle */
    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isReady() {
        return false; // Make me ready!
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}
