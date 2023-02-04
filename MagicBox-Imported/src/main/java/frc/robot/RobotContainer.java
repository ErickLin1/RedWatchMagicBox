// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commandgroups.Intake;
import frc.robot.commands.ChangeColor;
import frc.robot.commands.CheckObjectForColorChange;
import frc.robot.commands.EjectItem;
import frc.robot.commands.IntakeItem;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  // Motor Controller Definition
  // private final DrivetrainSparks m_drivetrainSparks;
  // private final DrivetrainTalons m_drivetrainTalons;

  // Controller Definition
  // private final XboxController m_sparkdriver = new XboxController(kSparkControllerPort);
  private final XboxController m_weaponsController = new XboxController(kTalonControllerPort);

  // Subsystems Definition
  private final Lights m_lights;
  private final Gripper m_gripper;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems Instantiation
    m_gripper = new Gripper();
    m_lights = new Lights();

    // Setting default commands
    m_lights.setDefaultCommand(new CheckObjectForColorChange(m_lights, m_gripper));

    SmartDashboard.putData(CommandScheduler.getInstance());
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_weaponsController, Button.kStart.value).toggleOnTrue(new Intake(m_gripper));
    new JoystickButton(m_weaponsController, Button.kBack.value).whenHeld(new EjectItem(m_gripper));
    
    new JoystickButton(m_weaponsController, Button.kLeftStick.value).whenPressed(new ChangeColor(m_lights, kYellowCone));
    new JoystickButton(m_weaponsController, Button.kRightStick.value).whenPressed(new ChangeColor(m_lights, kPurpleCube));
    new JoystickButton(m_weaponsController, Button.kA.value).whenPressed(new ChangeColor(m_lights, kParty));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
