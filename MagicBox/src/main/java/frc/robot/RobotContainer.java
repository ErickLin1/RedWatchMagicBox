// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.differentialDrive;
import frc.robot.commands.differentialDriveSparks;
import frc.robot.commands.differentialDriveTalons;
import frc.robot.commands.toggleSolenoid;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSparks;
import frc.robot.subsystems.DrivetrainTalons;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Drivetrain m_drivetrain;
  private DrivetrainSparks m_drivetrainSparks;
  private DrivetrainTalons m_drivetrainTalons;

  private final XboxController m_driver = new XboxController(Constants.kDriverControllerPort);

  private final Climber m_climber;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Comment out the code to enable/disable certain motors
    // Only one drivetrain subsystem can be active at a time

    // Set up Drivetrian (SPARKS AND TALONS)
    m_drivetrain = new Drivetrain();
    m_drivetrain.setDefaultCommand(
      new differentialDrive(() -> m_driver.getRightTriggerAxis(), () -> m_driver.getLeftTriggerAxis(), 
      () -> m_driver.getLeftY(), () -> m_driver.getRightY(), m_drivetrain));
    
    // Drivetrain for Sparks ONLY
    // m_drivetrainSparks = new DrivetrainSparks();
    // m_drivetrainSparks.setDefaultCommand(
    //   new differentialDriveSparks(() -> m_driver.getRightTriggerAxis(), () -> m_driver.getLeftTriggerAxis(), 
    //   () -> m_driver.getLeftY(), () -> m_driver.getRightY(), m_drivetrainSparks));

    // // Drivetrain for Talons ONLY
    // m_drivetrainTalons = new DrivetrainTalons();
    // m_drivetrainTalons.setDefaultCommand(
    //   new differentialDriveTalons(() -> m_driver.getRightTriggerAxis(), () -> m_driver.getLeftTriggerAxis(), 
    //   () -> m_driver.getLeftY(), () -> m_driver.getRightY(), m_drivetrainTalons));

    // Set up pneumatics and solenoids
    m_climber = new Climber();
    m_climber.setDefaultCommand(new toggleSolenoid(m_climber, m_driver));
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
