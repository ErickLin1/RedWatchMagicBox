package frc.robot.CustomShuffleboard;

import java.lang.reflect.Field;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class CustomControlpanel {
    private ArrayList<SubsystemStatus> m_statuses = new ArrayList<SubsystemStatus>();

    public CustomControlpanel(SubsystemStatus subsystem) {
        Field[] fields;
        try {
            fields = Class.forName(subsystem.getClass().getCanonicalName()).getFields();
        } catch (SecurityException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        } catch (ClassNotFoundException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        }
        
        m_statuses.stream().map(subsystem::getClass().getName());

        for (int i = 0; i < fields.length; i++) {
            
        }        
    }
}
