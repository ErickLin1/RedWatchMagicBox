// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainTalons;

public class differentialDriveTalons extends CommandBase {

  private final DrivetrainTalons m_drivetrain;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;

  /** Creates a new differentialDrive. */
  public differentialDriveTalons(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DrivetrainTalons subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = subsystem;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_leftSpeed.getAsDouble(), m_rightSpeed.getAsDouble(), false);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
