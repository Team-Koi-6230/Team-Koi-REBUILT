package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class feederSubsystem extends SubsystemBase {
    private final SparkMax m_SparkMax;
    private final double power;

    /** Creates a new ExampleSubsystem. */
    public feederSubsystem(double power) {
        this.power = power;
        m_SparkMax = new SparkMax(0, MotorType.kBrushless);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::runOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            
        });
    }

    public void setWheelVoltage() {
        m_SparkMax.setVoltage(power);
    }

    /**
     * An example method querying a boolean state of the subsystem
     * (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
    }
}
