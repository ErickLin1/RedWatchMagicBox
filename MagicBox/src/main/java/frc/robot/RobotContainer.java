// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.differentialDriveSparks;
import frc.robot.commands.differentialDriveTalons;
import frc.robot.commands.toggleSolenoid;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSparks;
import frc.robot.subsystems.DrivetrainTalons;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  private final DrivetrainSparks m_drivetrainSparks;
  private final DrivetrainTalons m_drivetrainTalons;

  private final XboxController m_sparkdriver = new XboxController(Constants.kSparkControllerPort);
  private final XboxController m_talondriver = new XboxController(Constants.kTalonControllerPort);

  private final Climber m_climber;

  private final ShuffleboardTab m_ShuffleboardTab = Shuffleboard.getTab(Constants.kShuffleboardTab);
  private final ShuffleboardLayout m_ShuffleboardLayout = m_ShuffleboardTab.getLayout("Motor Controls", BuiltInLayouts.kList)
  .withPosition(3, 0)
  .withSize(3, 3);
  private NetworkTableEntry LeftMotorSpeed = m_ShuffleboardLayout.add("Left Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
  private NetworkTableEntry RightMotorSpeed = m_ShuffleboardLayout.add("Right Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Drivetrain for Sparks
    m_drivetrainSparks = new DrivetrainSparks();
    m_drivetrainSparks.setDefaultCommand(
      new differentialDriveSparks(() -> -m_sparkdriver.getLeftY(), () -> -m_sparkdriver.getRightY(), m_drivetrainSparks));

    // // Drivetrain for Talons
    m_drivetrainTalons = new DrivetrainTalons();
    m_drivetrainTalons.setDefaultCommand(
      new differentialDriveTalons(() -> -m_talondriver.getLeftY(), () -> -m_talondriver.getRightY(), m_drivetrainTalons));

    // Set up pneumatics and solenoids
    m_climber = new Climber();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Add button for each controller to toggle solenoid
    new JoystickButton(m_sparkdriver, Button.kY.value).whenPressed(new toggleSolenoid(m_climber));
    new JoystickButton(m_talondriver, Button.kY.value).whenPressed(new toggleSolenoid(m_climber));

    // Shuffleboard buttons

    // Enables or disables the solenoid
    m_ShuffleboardTab.add("Toggle Solenoid", new toggleSolenoid(m_climber))
      .withPosition(3, 3)
      .withSize(3, 1);
    // Turns on the motors and reads the shuffleboard's motor speed values
    m_ShuffleboardLayout.add("Run Motors", new differentialDriveSparks(() -> LeftMotorSpeed.getDouble(0), () -> RightMotorSpeed.getDouble(0), m_drivetrainSparks));
  }

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
