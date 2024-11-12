package frc.robot;



//package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




DigitalInput topLimitSwitch = new DigitalInput(0);
DigitalInput bottomLimitSwitch = new DigitalInput(1);
Joystick stick = new Joystick(0);

public class LimitSwitchMotor extends TimedRobot {
    // neos motor1 = new neo(0);
    Joystick stick;
    DigitalInput limitSwitch;
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    SparkMaxLimitSwitch forward_limit_L;
    SparkMaxLimitSwitch reverse_limit_L;
    SparkMaxLimitSwitch forward_limit_R;
    SparkMaxLimitSwitch reverse_limit_R;


        


    @Override
    public void robotInit() {
        leftMotor = new CANSparkMax(2, MotorType.kBrushless);
        rightMotor = new CANSparkMax(2, MotorType.kBrushless);

        forward_limit_L.enableLimitSwitch(false);
        reverse_limit_L.enableLimitSwitch(false);
        forward_limit_R.enableLimitSwitch(false);
        reverse_limit_R.enableLimitSwitch(false);
        
        
    }
   
    public void teleopPeriodic() {
    
    }

    public void autonomousPeriodic() {
    
    }
}