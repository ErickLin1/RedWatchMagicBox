// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ChangeLEDColor;
import frc.robot.commands.NewChangeLEDColor;
import frc.robot.commands.NewCheckObjectColor;
import frc.robot.commands.CheckObjectColor;
import frc.robot.commands.EjectItem;
import frc.robot.commands.IntakeItem;
import frc.robot.commands.PickUpItem;
import frc.robot.commands.StopGripper;
import frc.robot.commands.TurnLightsBlue;
import frc.robot.commands.cycleLightsLeft;
import frc.robot.commands.cycleLightsRight;
import frc.robot.commands.potToLights;
import frc.robot.subsystems.NewLights;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.Gripper;
// import frc.robot.subsystems.Lights;
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

  // private final XboxController m_sparkdriver = new XboxController(kSparkControllerPort);
  private final XboxController m_talondriver = new XboxController(kTalonControllerPort);

  // private final Climber m_climber;
  private final ColorDetection m_color;

  // private final Lights m_light;
  private final NewLights m_newlight;
  // private final MeasuringPotentiometer m_pot;
  // private final Gripper m_gripper;

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
    // m_gripper = new Gripper();
    // m_climber = new Climber();
    m_color = new ColorDetection();
    // m_light = new Lights(); 
    m_newlight = new NewLights();
    m_color.setDefaultCommand(new NewCheckObjectColor(m_color, m_newlight));

    // Sets up the control panel
    //new ControlPanel(m_climber, m_drivetrainSparks, m_drivetrainTalons);

    // Sets up Color Sensor
    new ColorDetection();

    // m_pot = new MeasuringPotentiometer();
    // m_light.setDefaultCommand(new potToLights(m_pot, m_light));
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
    // new JoystickButton(m_talondriver, Button.kA.value).whenPressed(new TurnLightsBlue(m_light));
    // new JoystickButton(m_talondriver, Button.kLeftBumper.value).whenPressed(new cycleLightsLeft(m_light));
    // new JoystickButton(m_talondriver, Button.kRightBumper.value).whenPressed(new cycleLightsRight(m_light, m_pot));
    // new JoystickButton(m_talondriver, Button.kX.value).whenPressed(new ChangeLEDColor(m_light, kPurpleCube));
    // new JoystickButton(m_talondriver, Button.kY.value).whenPressed(new ChangeLEDColor(m_light, kYellowCone));
    // new JoystickButton(m_talondriver, Button.kStart.value).whenPressed(new IntakeItem(m_gripper));
    // new JoystickButton(m_talondriver, Button.kBack.value).whenPressed(new EjectItem(m_gripper));
    // new JoystickButton(m_talondriver, Button.kLeftStick.value).whenPressed(new ChangeLEDColor(m_light, kYellowCone));
    // new JoystickButton(m_talondriver, Button.kRightStick.value).whenPressed(new ChangeLEDColor(m_light, kPurpleCube));
    new JoystickButton(m_talondriver, Button.kX.value).onTrue(new NewChangeLEDColor(m_newlight, 101, 15, 140));
    new JoystickButton(m_talondriver, Button.kY.value).onTrue(new NewChangeLEDColor(m_newlight, 255, 255, 0));
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
