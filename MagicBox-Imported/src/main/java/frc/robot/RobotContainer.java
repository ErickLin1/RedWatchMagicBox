// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.commands.ArmMovement;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PotExtend;
import frc.robot.commands.lightUpMaxExtend;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.motor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final motor m_Arm;
  private final GenericHID j_stick = new GenericHID(Constants.kJoystickPort);
  private final Light m_led;

  XboxController controller1 = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Arm = new motor();
    m_led = new Light();
    // Configure the button bindings

    m_Arm.setDefaultCommand(new ArmMovement (controller1.getLeftY(), m_Arm));
    m_led.setDefaultCommand(new lightUpMaxExtend(m_led, m_Arm));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(controller1, Button.kA.value).onTrue(new PotExtend(15, m_Arm));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
