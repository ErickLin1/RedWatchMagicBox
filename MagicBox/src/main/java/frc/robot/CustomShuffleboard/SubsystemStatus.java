package frc.robot.CustomShuffleboard;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemStatus {
    private final ShuffleboardLayout m_layout;
    private final SubsystemBase m_subsystem;
    private ArrayList<SuppliedValueWidget<String>> m_values;

    public SubsystemStatus(ShuffleboardLayout layout, SubsystemBase subsystem) {
        m_layout = layout;
        m_subsystem = subsystem;
    }
}
