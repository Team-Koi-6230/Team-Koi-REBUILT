package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeRollerSubsytem extends SubsystemBase {
    public enum IntakeRollerState {
        SPINNING,
        IDLE
    }

    private final SparkMax m_motor;
    private IntakeRollerState state;

    public IntakeRollerSubsytem() {
        m_motor = new SparkMax(Constants.IntakeRollerConstants.kMotorID, MotorType.kBrushless);
        state = IntakeRollerState.IDLE;
    }

    public Command rollerSpinCommand(double voltage) {
        return runOnce(() -> {
            setVoltage(voltage);
        });
    }

    public void setVoltage(double voltage) {
        state = voltage != 0 ? IntakeRollerState.SPINNING : IntakeRollerState.IDLE;
        m_motor.setVoltage(voltage);
    }

    public IntakeRollerState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
