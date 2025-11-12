package frc.robot;

import java.io.File;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    // Climber Constants
    public static class ClimberConstants {
        public static final int CLIMBER_CAN_ID = 13;
        public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
        public static final double CLIMBER_SPEED = 1.0;
    }

    // Drivetrain Constants
    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = Units.feetToMeters(18.5); // Theoretically: ~19.2-19.8, Real: 15-17?
        public static final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    }

}
