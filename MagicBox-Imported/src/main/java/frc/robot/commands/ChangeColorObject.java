// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Gripper;

public class ChangeColorObject extends CommandBase {
  private final Lights m_blinkyblinky;
  private final Gripper m_grippygrippy;

  /** Creates a new ChangeColorObject. */
  public ChangeColorObject(Lights lights, Gripper gripper) {
    m_blinkyblinky = lights;
    m_grippygrippy = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_grippygrippy.isPurple())
      m_blinkyblinky.setCube();
    
    else if (m_grippygrippy.isYellow())
      m_blinkyblinky.setCone();
    
    else
      m_blinkyblinky.setDisabledColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
