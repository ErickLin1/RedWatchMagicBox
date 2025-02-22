// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.GripperConstants.*;

public class PickUpItem extends CommandBase {
  /** Creates a new PickUpItem. */

private final Gripper m_gripper;
// private final light m_lights

  public PickUpItem(Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.

m_gripper = gripper;
addRequirements(gripper);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

m_gripper.intakeGripper();

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
