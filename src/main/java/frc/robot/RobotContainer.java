// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  final XboxController controller = new XboxController(USBConstants.DRIVER_CONTROLLER_PORT);

  private void calibrate(){
    System.out.println("Gyro is calibrating...");
    driveSubsystem.calibrateGyro();
  }
  public RobotContainer() {
    calibrate();
    configureButtonBindings();

    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      driveSubsystem.arcadeDrive(
          controller.getLeftY(),
          controller.getRightX());
    }, driveSubsystem));

  }

  private void configureButtonBindings() {
   
  }


  public Command getAutonomousCommand() {
  
    return null;
  }
}
