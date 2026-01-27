package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArmSubsystem;

/* You're allowed to refer to this command as the thug shake, please do */
public class ArmShakeCommand extends Command {
    private final IntakeArmSubsystem intakeArmSubsystem;
    private final Timer timer;
    
    private boolean isWaiting;
    // if the shake is opening or closing
    private enum shakeState   
    {
        TO_MIN,
        TO_MAX
    }
    private shakeState state;

    public ArmShakeCommand(IntakeArmSubsystem intakeArmSubsystem) {
        this.intakeArmSubsystem = intakeArmSubsystem;
        addRequirements(intakeArmSubsystem);
        
        timer = new Timer();
        state = shakeState.TO_MAX;
    }
    

    @Override
    public void initialize() {
        timer.start();  // first start of the timer
        isWaiting = false; 

        // start the shake by openeing to the max shake angle
        state = shakeState.TO_MAX;
        intakeArmSubsystem.setAngle(IntakeArmConstants.kShakeMax);
    }
     
    @Override
    public void execute() {
        // if the arm is in the max shake angle, change the direction to move towards the min shake angle
        if (intakeArmSubsystem.getAngle() >= IntakeArmConstants.kShakeMax && state == shakeState.TO_MAX) {
            changeDirection(IntakeArmConstants.kShakeMin);
        }

        // if the arm is in the min shake angle, change the direction to move towards the max shake angle
        if (intakeArmSubsystem.getAngle() <= IntakeArmConstants.kShakeMin && state == shakeState.TO_MIN) {
            changeDirection(IntakeArmConstants.kShakeMax);
        }
    }

    private void changeDirection(double angle) {
        // only have slight delay when pushing the fuel in (idfk I think ts might help)
        if (!isWaiting && state == shakeState.TO_MIN) { // check if the shake is in delay and approaching minimum shake angle
            timer.restart();
            isWaiting = true;
        }
        
        // change the direction if after a few moments of delay
        if (timer.get() > IntakeArmConstants.kShakeDelay) {
            intakeArmSubsystem.setAngle(angle);
            isWaiting = false;
            state = state == shakeState.TO_MAX ? shakeState.TO_MIN : shakeState.TO_MAX;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSubsystem.CloseArm();
    }
} 