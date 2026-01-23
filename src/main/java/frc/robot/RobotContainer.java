package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ScoreCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GameDataSubsystem;
import frc.robot.utils.RumbleSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final RumbleSubsystem rumbleSubsystem = new RumbleSubsystem(m_driverController, m_operatorController);
  @SuppressWarnings("unused") // it's a periodic only subsystem
  private final GameDataSubsystem gameDataSubsystem = new GameDataSubsystem(rumbleSubsystem);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      RobotBase.isSimulation() ? "swerve-sim" : "swerve"));

  
    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * (m_driverController.rightBumper().getAsBoolean() ? -0.4 : -1),
      () -> m_driverController.getLeftX() * (m_driverController.rightBumper().getAsBoolean() ? -0.4 : -1))
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
      .deadband(OperatorConstants.kDeadband)
      .scaleTranslation(1)
      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> m_driverController.getRightX() * -0.7,
          () -> m_driverController.getRightY() * -0.7)
      .headingWhile(true);

  

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command scoreCommand = new ScoreCommand(shooterSubsystem);
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    m_driverController.rightBumper().whileTrue(drivebase.driveRelativeToHub(driveAngularVelocity));
    m_driverController.x().whileTrue(scoreCommand);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
