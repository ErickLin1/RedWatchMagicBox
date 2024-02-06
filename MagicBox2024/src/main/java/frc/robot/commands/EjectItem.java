// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.GripperConstants.*;

public class EjectItem extends Command {

  private final Gripper m_gripper;

  /** Creates a new EjectItem. */
  public EjectItem(Gripper gripper) {
    m_gripper = gripper;

addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Gripper.m_gripper_direction != "eject")
      m_gripper.ejectGripper();
    
    else
      m_gripper.stopGripper();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
