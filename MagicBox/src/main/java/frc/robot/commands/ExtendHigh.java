// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
public class ExtendHigh extends CommandBase {

  private final MeasuringPotentiometer m_pot;
  private double neededPot=10;
  private final SingleSpark m_spark;
  private boolean isReverse = false;
  private double speed = 0.5;
  /** Creates a new ExtendHigh. */
  public ExtendHigh(boolean reverse, double potLength, MeasuringPotentiometer potentiometer, SingleSpark spark) {
    m_pot = potentiometer;
    m_spark = spark;
    isReverse = reverse;
    neededPot = potLength;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_spark);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spark.encoderReset(m_spark.m_leftEncoder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean rev = false;
    if(isReverse){
      rev = (m_pot.pot_val > neededPot);
    }else{
      rev = (m_pot.pot_val < neededPot);
    }
    if (rev) {
      m_spark.leftSpark.set(speed);
    }
    else {
      m_spark.leftSpark.set(0);
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
