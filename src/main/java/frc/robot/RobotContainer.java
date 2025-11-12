package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {

    // Subsystems
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(DrivetrainConstants.swerveJsonDirectory);

    // Joysticks
    final CommandXboxController controllerXbox = new CommandXboxController(0);

    // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                  () -> controllerXbox.getLeftY() * -1,
                                                                  () -> controllerXbox.getLeftX() * -1)
                                                              .withControllerRotationAxis(() -> controllerXbox.getRightX() * -1)
                                                              .deadband(0.2)
                                                              .scaleTranslation(0.8)
                                                              .scaleRotation(0.8)
                                                              .allianceRelativeControl(true);

    // Constructor
    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // Climb Controls
        controllerXbox.rightBumper().whileTrue(new ClimberCommand(climberSubsystem, ClimberConstants.CLIMBER_SPEED));
        controllerXbox.leftBumper().whileTrue(new ClimberCommand(climberSubsystem, -ClimberConstants.CLIMBER_SPEED));
 
    }

    // Set drive Controls For Teleop
    public void setSwerveDefaultCommand() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
    }

}
