// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RevToSpeed extends CommandBase {

  Shooter m_shooter;
  double m_motorPower;
  double currentRPM;
  double m_TargetRPM;
  double increment;
  double error;

  /** Creates a new RevToSpeed. */
  public RevToSpeed(double rpm, Shooter shooter) {

    m_TargetRPM = rpm;
    m_shooter = shooter;

    double m_motorPower = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.topMotor.set(0);
    m_shooter.encoderReset(m_shooter.m_topEncoder);

    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRPM = m_shooter.getEncoderVelocity(m_shooter.m_topEncoder);
    error = m_TargetRPM - currentRPM;

    increment = error/100000;
    m_shooter.increment = increment;
    m_motorPower += increment;
    m_shooter.motorPower = m_motorPower;

    m_shooter.topMotor.set(m_motorPower);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.flyWheelSpeedAfterRev = m_motorPower;


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(currentRPM - m_TargetRPM) <= 0.005;
  }
}
