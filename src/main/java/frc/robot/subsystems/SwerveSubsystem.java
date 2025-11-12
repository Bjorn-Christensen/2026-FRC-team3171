package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.Constants.DrivetrainConstants;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory) {

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivetrainConstants.MAX_SPEED); // Initialize SwerveDrive Object
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.

    }

    // Drive the robot given a chassis field oriented velocity.
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

}
