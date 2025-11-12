// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // Instantiate the RobotContainer
    robotContainer = new RobotContainer();

    // Cuts out unnecessary error logs when simulating robot
    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

  }

  @Override
  public void robotPeriodic() {
    // Required to run command based code
    CommandScheduler.getInstance().run();
 }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // Set default commands once the match begins
    robotContainer.setSwerveDefaultCommand();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode
    CommandScheduler.getInstance().cancelAll();

    // Set default commands once the match begins
    robotContainer.setSwerveDefaultCommand();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
    
  }
}