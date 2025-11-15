package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Elevator.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {

  // Subsystems
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(DrivetrainConstants.swerveJsonDirectory);

  // Joysticks
  final CommandXboxController controllerXbox = new CommandXboxController(0);

  // Telemetry
  private final Telemetry telemetry = new Telemetry(elevatorSubsystem, climberSubsystem);

  // Auton Chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // Subsystem requirements required for commands writting direct within subsystem class
  private final Set<Subsystem> swerveReq = Set.of(swerveSubsystem);

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

    // Register named commands for PathPlanner
    NamedCommands.registerCommand("PrecisionScoreLeft", 
        Commands.defer(() -> swerveSubsystem.precisionLineUp(true), Set.of(swerveSubsystem)));
    NamedCommands.registerCommand("PrecisionScoreRight", 
        Commands.defer(() -> swerveSubsystem.precisionLineUp(false), Set.of(swerveSubsystem)));

    // Build chooser after NamedCommands so event markers have something to call
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser("Test"));
  }

  private void configureButtonBindings() {
    // Climb Controls
    controllerXbox.rightBumper().whileTrue(new ClimberCommand(climberSubsystem, -ClimberConstants.CLIMBER_SPEED));
    controllerXbox.leftBumper().whileTrue(new ClimberCommand(climberSubsystem, -ClimberConstants.CLIMBER_SPEED));

    // Elevator Controls
    controllerXbox.rightTrigger(ControllerConstants.RIGHT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, controllerXbox));
    controllerXbox.leftTrigger(ControllerConstants.LEFT_TRIGGER_DEADZONE).whileTrue(new ElevatorJoystickCommand(elevatorSubsystem, controllerXbox));
    controllerXbox.a().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_ONE));       
    controllerXbox.b().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_TWO));
    controllerXbox.x().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.POSITION_THREE));
    // controllerXbox.leftBumper().onTrue(new ElevatorPIDCommand(elevatorSubsystem, ElevatorConstants.LOAD_STATION_POSITION));

    // Auto Drive Controls
    controllerXbox.povRight().onTrue(Commands.defer(() -> swerveSubsystem.driveToReef(false), swerveReq));
    controllerXbox.povLeft().onTrue(Commands.defer(() -> swerveSubsystem.driveToReef(true), swerveReq));
    controllerXbox.povDown().onTrue(Commands.defer(() -> swerveSubsystem.driveToHumanLoad(), swerveReq));

  }

  // Auton chooser called on Autonomous Init
  public Command getAutonomousCommand() {
    Command selected = autoChooser.get();
    if(selected != null) {
      String autoName = selected.getName();
      try {
        var paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        if (!paths.isEmpty()) {
          PathPlannerPath firstPath = paths.get(0);
          Pose2d startPose = firstPath.getPathPoses().get(0);
          swerveSubsystem.primeStartingPose(startPose);
        }
      } catch (Exception e) {
        System.err.println("primeStartingPose failed: " + e.getMessage());
      }
    }

    return selected;
  }

  // Set drive Controls For Teleop
  public void setSwerveDefaultCommand() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
  }

  // Set drive motor brakes
  public void setDriveMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }

  // Allows elevator to be dropped to base position while disabled
  public void setElevatorMotorBrake(boolean brake) {
    elevatorSubsystem.setMotorBrake(brake);
  }

  // Runs Live PID Tuning
  public void periodic() {
    telemetry.periodic();
  }

}
