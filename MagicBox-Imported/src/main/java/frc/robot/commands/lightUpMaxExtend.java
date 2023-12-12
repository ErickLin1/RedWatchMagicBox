// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.motor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class lightUpMaxExtend extends CommandBase {
  /** Creates a new lightUpMaxExtend. */
  private final Light ledController;
  private final motor m_Arm;

  public lightUpMaxExtend(Light controller,  motor armMotor) {
    ledController = controller;
    m_Arm = armMotor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Arm.getPosition() == 50) {  
      ledController.turnLEDOn();
    }
    else if (m_Arm.getPosition() != 50) {
      ledController.turnLEDOff();  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
