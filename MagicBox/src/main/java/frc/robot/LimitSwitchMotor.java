package frc.robot;

// need to figure out how to import NEOs
// import edu.wpi.first.wpilibj.neos;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

DigitalInput topLimitSwitch = new DigitalInput(0);
DigitalInput bottomLimitSwitch = new DigitalInput(1);
Joystick stick = new Joystick(0);

public class LimitSwitchMotor {
    // neos motor1 = new neo(0);
    Joystick stick;
    DigitalInput limitSwitch;

    public void robotInit() {
        
        
    }
   
    public void autonomousInit() {
    
    }

    public void autonomousPeriodic() {
    
    }
}