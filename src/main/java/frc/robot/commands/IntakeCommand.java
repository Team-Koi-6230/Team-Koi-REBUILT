package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsytem;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;
import frc.robot.utils.RumbleSubsystem;

public class IntakeCommand extends Command {

    private final IntakeArmSubsystem arm;
    private final IntakeRollerSubsytem rollers;
    private final RumbleSubsystem rumble;

    private boolean firstTimeReady = true;

    public IntakeCommand(IntakeArmSubsystem arm, IntakeRollerSubsytem rollers, RumbleSubsystem rumble) {
        this.arm = arm;
        this.rollers = rollers;
        this.rumble = rumble;
        addRequirements(arm, rollers);
    }

    @Override
    public void execute() {
        arm.OpenArm();
        if (arm.getState() != IntakeArmState.OPEN)
            return;
        if (firstTimeReady) {
            rumble.rumble(Constants.IntakeRollerConstants.kIntakeReadyRumble);
            firstTimeReady = false;
        }
        rollers.setVoltage(Constants.IntakeRollerConstants.kIntakePower);
    }

    public void end(boolean interrupted) {
        rollers.setVoltage(0);
        firstTimeReady = true;
    }
}
