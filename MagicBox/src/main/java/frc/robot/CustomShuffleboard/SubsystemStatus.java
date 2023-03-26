package frc.robot.CustomShuffleboard;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemStatus {
    private final SubsystemBase m_subsystem;
    private ArrayList<SuppliedValueWidget<String>> m_values = new ArrayList<SuppliedValueWidget<String>>();

    public SubsystemStatus(ShuffleboardLayout layout, SubsystemBase subsystem) {
        m_subsystem = subsystem;

        Field[] fields;
        try {
            fields = Class.forName(m_subsystem.getClass().getCanonicalName()).getFields();
        } catch (SecurityException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        } catch (ClassNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        }
        
        for (int i = 0; i < fields.length; i++) {
            try {
                layout.addString(fields[i].getName(), () -> (String) m_subsystem
                    .getClass()
                    .getDeclaredField(
                        fields[i]
                        .getName()
                    )
                    .get(subsystem)
                );
            } catch (IllegalAccessException e) {
                return;
            }
        }
    }
}