// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSparks;
import frc.robot.subsystems.RPMSpeedPID;

public class RunMotorsRPM extends CommandBase {
  /** Creates a new RunMotorsRPM. */
  private final DoubleSupplier leftMotorRPM;
  private final DoubleSupplier rightMotorRPM;
  private final RPMSpeedPID m_LeftPID;
  private final RPMSpeedPID m_RightPID;
  public RunMotorsRPM(DoubleSupplier leftRPM, DoubleSupplier rightRPM, DrivetrainSparks drivetrain) {
    m_LeftPID = new RPMSpeedPID(drivetrain.leftSpark);
    m_RightPID = new RPMSpeedPID(drivetrain.rightSpark);

    leftMotorRPM = leftRPM;
    rightMotorRPM = rightRPM;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LeftPID.setSetpoint(leftMotorRPM.getAsDouble());
    m_RightPID.setSetpoint(rightMotorRPM.getAsDouble());

    m_LeftPID.enable();
    m_RightPID.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LeftPID.disable();
    m_RightPID.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
