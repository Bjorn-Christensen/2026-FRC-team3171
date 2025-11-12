package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    // Hardware
    private final SparkMax climberMotor; // Declare the motor controller

    // Constructor
    public ClimberSubsystem() {
        
        // Initialize the motor controller
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_CAN_ID, ClimberConstants.MOTOR_TYPE);

    }

    // Manual climber control
    public void setMotor(double speed) {
        climberMotor.set(speed);
    }

}