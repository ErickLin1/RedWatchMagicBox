package frc.robot.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemStatus extends ShuffleboardLayout{
    
    private final SubsystemBase m_subsystem;

    public SubsystemStatus(SubsystemBase subsystem) {
        super(, m_title, m_title);
        m_subsystem = subsystem;
    }
}
