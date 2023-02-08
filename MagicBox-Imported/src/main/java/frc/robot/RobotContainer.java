// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AprilTagMode;
import frc.robot.commands.ReflectiveTapeMode;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.differentialDriveSparks;
// import frc.robot.commands.differentialDriveTalons;
import frc.robot.commands.toggleSolenoid;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorDetection;
// import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.DrivetrainTalons;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.ControllerConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController m_driver = new XboxController(Constants.ControllerConstants.kDriverPort);
  private final Vision m_vision = new Vision();
  private final Drivetrain m_drivetrain = new Drivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driver, Button.kA.value).onTrue(new AprilTagMode(m_vision));
    new JoystickButton(m_driver, Button.kB.value).onTrue(new ReflectiveTapeMode(m_vision));
    new JoystickButton(m_driver, Button.kX.value).onTrue(new VisionAlign(m_drivetrain, m_vision));
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
