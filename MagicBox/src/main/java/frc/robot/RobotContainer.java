// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ChangeLEDColor;
import frc.robot.commands.TurnLightsBlue;
import frc.robot.commands.cycleLightsLeft;
import frc.robot.commands.cycleLightsRight;
import frc.robot.commands.potToLights;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MeasuringPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.ControllerConstants.*;
import static frc.robot.Constants.LightConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSparks m_drivetrainSparks;
  // private final DrivetrainTalons m_drivetrainTalons;

  private final XboxController m_sparkdriver = new XboxController(kSparkControllerPort);
  private final XboxController m_talondriver = new XboxController(kTalonControllerPort);

  private final Lights m_light;
  private final MeasuringPotentiometer m_pot;

  // private final Climber m_climber;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Drivetrain for Sparks
    // m_drivetrainSparks = new DrivetrainSparks();
    // m_drivetrainSparks.setDefaultCommand(
    //   new differentialDriveSparks(() -> -m_sparkdriver.getLeftY(), () -> -m_sparkdriver.getRightY(), m_drivetrainSparks));

    // // Drivetrain for Talons
    // m_drivetrainTalons = new DrivetrainTalons();
    // m_drivetrainTalons.setDefaultCommand(
    //   new differentialDriveTalons(() -> -m_talondriver.getLeftY(), () -> -m_talondriver.getRightY(), m_drivetrainTalons));

    // Sets up pneumatics and solenoids
    // m_climber = new Climber();

    // Sets up the control panel
    // new ControlPanel(m_climber, m_drivetrainSparks, m_drivetrainTalons);

    // Sets up Color Sensor
    // new ColorDetection();

    m_light = new Lights();
    m_pot = new MeasuringPotentiometer();
    m_light.setDefaultCommand(new potToLights(m_pot, m_light));
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
    // new JoystickButton(m_sparkdriver, Button.kY.value).whenPressed(new toggleSolenoid(m_climber));
    // new JoystickButton(m_talondriver, Button.kY.value).whenPressed(new toggleSolenoid(m_climber));
    new JoystickButton(m_sparkdriver, Button.kA.value).whenPressed(new TurnLightsBlue(m_light));
    new JoystickButton(m_sparkdriver, Button.kLeftBumper.value).whenPressed(new cycleLightsLeft(m_light));
    new JoystickButton(m_sparkdriver, Button.kRightBumper.value).whenPressed(new cycleLightsRight(m_light, m_pot));
    new JoystickButton(m_sparkdriver, Button.kX.value).whenPressed(new ChangeLEDColor(m_light, kPurpleCube));
    new JoystickButton(m_sparkdriver, Button.kY.value).whenPressed(new ChangeLEDColor(m_light, kYellowCone));

  }

  /**
   * Use this to pass the autcycleLightsRightonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
