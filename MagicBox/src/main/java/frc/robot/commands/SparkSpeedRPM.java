// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSparks;

public class SparkSpeedRPM extends CommandBase {
  /** Creates a new SparkSpeedRPM. */
  private final PIDController leftPID;
  // private final PIDController rightPID;

  private double leftFinalSpeed;
  private double rightFinalSpeed;

  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;
  private final DrivetrainSparks m_drivetrain;
  public SparkSpeedRPM(DoubleSupplier lspeed, DoubleSupplier rSpeed, DrivetrainSparks drivetrain) {
    leftSpeed = lspeed;
    rightSpeed = rSpeed;
    m_drivetrain = drivetrain;

    leftPID = new PIDController(0, 0.001, 0);
    // rightPID = new PIDController(0, 0.001, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftPID.setTolerance(10);
    // rightPID.setTolerance(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!leftPID.atSetpoint()) {
      leftFinalSpeed = leftPID.calculate(m_drivetrain.getLeftSpeed(), leftSpeed.getAsDouble());
      m_drivetrain.setLeftSpeed(leftFinalSpeed);
      // m_drivetrain.setRightSpeed(rightPID.calculate(m_drivetrain.getRightSpeed(), rightSpeed.getAsDouble()));
    } else {
      m_drivetrain.setLeftSpeed(leftFinalSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leftPID.reset();
    // rightPID.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
