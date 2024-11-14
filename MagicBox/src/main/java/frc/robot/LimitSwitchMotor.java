package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class LimitSwitchMotor extends TimedRobot {
    // neos motor1 = new neo(0);
    DigitalInput topLimitSwitch = new DigitalInput(0);
    DigitalInput bottomLimitSwitch = new DigitalInput(1);
    Joystick stick = new Joystick(0);
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
        
        forward_limit_L = leftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        reverse_limit_L = leftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        forward_limit_R = rightMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        reverse_limit_R = rightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);


        forward_limit_L.enableLimitSwitch(false);
        reverse_limit_L.enableLimitSwitch(false);
        forward_limit_R.enableLimitSwitch(false);
        reverse_limit_R.enableLimitSwitch(false);

        if (forward_limit_L.isLimitSwitchEnabled()) {
            leftMotor.set(0.5);
        }
        if (reverse_limit_L.isLimitSwitchEnabled()) {
            leftMotor.set(-0.5);
        }
        if (forward_limit_R.isLimitSwitchEnabled()) {
            rightMotor.set(0.5);
        }
        if (reverse_limit_R.isLimitSwitchEnabled()) {
            rightMotor.set(-0.5);
        }
        
        
    }
   
    public void teleopPeriodic() {
        forward_limit_L.enableLimitSwitch(SmartDashboard.getBoolean("Left Forward Limit Enabled", false));
        reverse_limit_L.enableLimitSwitch(SmartDashboard.getBoolean("Left Reverse Limit Enabled", false));
        forward_limit_R.enableLimitSwitch(SmartDashboard.getBoolean("Right Forward Limit Enabled", false));
        reverse_limit_R.enableLimitSwitch(SmartDashboard.getBoolean("Right Reverse Limit Enabled", false));
    }

    public void autonomousPeriodic() {
    
    }
}