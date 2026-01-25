package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RumbleSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final Superstructure superstructure;

  private final RumbleSubsystem rumbleSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeArmSubsystem intakeArmSubsystem;
  private final IntakeRollerSubsytem intakeRollerSubsytem;
  private final ClimberSubsystem climberSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SwerveSubsystem drivebase;

  private final SwerveInputStream driveAngularVelocity;
  private final SwerveInputStream driveDirectAngle;

  public RobotContainer() {
    // Controllers
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Superstructure & subsystems (lazy init)
    superstructure = Superstructure.getInstance();

    rumbleSubsystem = superstructure.getRumbleSubsystem();
    shooterSubsystem = superstructure.getShooterSubsystem();
    feederSubsystem = superstructure.getFeederSubsystem();
    intakeArmSubsystem = superstructure.getIntakeArmSubsystem();
    intakeRollerSubsytem = superstructure.getIntakeRollerSubsytem();
    climberSubsystem = superstructure.getClimberSubsystem();
    hoodSubsystem = superstructure.getHoodSubsystem();
    drivebase = superstructure.getDrivebase();

    // Input streams
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * (m_driverController.rightBumper().getAsBoolean() ? -0.4 : -1),
        () -> m_driverController.getLeftX() * (m_driverController.rightBumper().getAsBoolean() ? -0.4 : -1))
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
        .deadband(OperatorConstants.kDeadband)
        .scaleTranslation(1)
        .allianceRelativeControl(false);

    driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> m_driverController.getRightX() * -0.7,
            () -> m_driverController.getRightY() * -0.7)
        .headingWhile(true);

    configureBindings();
  }

  private void configureBindings() {
    if (RobotBase.isReal()) {
      configureBindingsReal();
    } else {
      configureBindingsSim();
    }
  }

  private void configureBindingsReal() {
    rumbleSubsystem.setControllers(m_driverController, m_operatorController);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command scoreCommand = new ScoreCommand(shooterSubsystem, hoodSubsystem, feederSubsystem);
    Command intakeCommand = new IntakeCommand(intakeArmSubsystem, intakeRollerSubsytem);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    m_driverController.rightBumper().whileTrue(drivebase.driveRelativeToHub(driveAngularVelocity));
    m_driverController.rightTrigger().whileTrue(scoreCommand);
    m_driverController.leftTrigger().whileTrue(intakeCommand);

    m_driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
  }

  private void configureBindingsSim() {
    shooterSubsystem.setTargetRPM(1000);

    m_driverController.a().whileTrue(
    Commands.run(() -> {
        double current = shooterSubsystem.getTargetRPM();
        if (!Double.isNaN(current)) {
            shooterSubsystem.setTargetRPM(current + 10);
        } else {
            shooterSubsystem.setTargetRPM(1000); // some default starting RPM
        }
    }, shooterSubsystem)
);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
