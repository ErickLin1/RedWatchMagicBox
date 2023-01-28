package frc.robot.commands;
import frc.robot.subsystems.TelescopingArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.TelescopingConstants.*;


public class Retract extends CommandBase {
    private final TelescopingArm m_Arm;

    /**
     * Decreases the height of the hangers
     * @param subsystem Hanger subsystem
     */

     public Retract(TelescopingArm subsystem) {
        // Use add requirements() here to declare subsystem dependencies.
        m_Arm = subsystem;
        addRequirements(m_Arm);
     }

     // Called when command is initially scheduled
     @Override
     public void initialize() {
        m_Arm.encoderReset(m_Arm.m_ArmEncoder);
     }   

     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
        m_Arm.turnMotor(m_Arm.m_ArmExtend, true);
     }

     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
        m_Arm.encoderReset(m_Arm.m_ArmEncoder);
     }

     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
        if (m_Arm.getArmDistance() > kArmSize) {
          return true;
        } else {
          return false;
        }
      }
}
