package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArm; 

public class ArmRunMotors extends CommandBase {

private final TelescopingArm m_Arm;
private final DoubleSupplier m_ArmSpeed;
private final CANSparkMax m_ArmMotor;

/**
 * Controls specific motors of the telescoping arm subsystem.
 * @param armSpeed 
 * @param ArmMotor Spark Max Motor
 */

public ArmRunMotors(DoubleSupplier armSpeed, CANSparkMax armMotor, TelescopingArm subsystem) {

// Use addRequirements() here to declare subsystem dependencies.
m_Arm = subsystem;
m_ArmSpeed = armSpeed;
m_ArmMotor = armMotor;

addRequirements(m_Arm);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
  m_ArmMotor.stopMotor(); 
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  // move motors of arm.
  m_ArmMotor.set(m_ArmSpeed.getAsDouble());
}

// Called once the command ends or is interrupted.

@Override
public void end(boolean interrupted) {
  // stop arm.
  m_ArmMotor.stopMotor();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  // command only stops when another is called
  return false;
 }
}