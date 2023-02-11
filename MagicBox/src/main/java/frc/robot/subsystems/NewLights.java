// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import static frc.robot.Constants.LightConstants.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;



public class NewLights extends SubsystemBase {
  /** Creates a new NewLights. */

  public final CANdle m_candle;
  private static final double kDisabled = 0;
  private final NetworkTable m_lightTable;
  private final Timer m_timeToSpeed = new Timer();
  private final ShuffleboardTab m_ShuffleboardTab;
  private final ShuffleboardLayout m_lightValues;
  private final int Ledcount = 32;
  public int count =0; 
  public int lCount = 1;
  public enum AnimationTypes {
    Twinkle;
  }
  private AnimationTypes m_currentAnimation;

  public NewLights() {

    m_ShuffleboardTab = Shuffleboard.getTab(Constants.kShuffleboardTab);
    m_lightValues = m_ShuffleboardTab.getLayout("Light Jawndess", BuiltInLayouts.kList);
    m_lightTable = NetworkTableInstance.getDefault().getTable("Light Statuses");
    CANdleConfiguration LEDConfig = new CANdleConfiguration();
    LEDConfig.statusLedOffWhenActive = false;
    LEDConfig.disableWhenLOS = false;
    LEDConfig.stripType = LEDStripType.RGB;
    LEDConfig.brightnessScalar = 0.5;
    LEDConfig.vBatOutputMode = VBatOutputMode.On;
    m_candle = new CANdle(kPhoenixDriverPort, "rio");
    m_candle.configAllSettings(LEDConfig, 100);
    setDefault();
    // m_candle.animate( new RainbowAnimation(1, 1, Ledcount));
    // m_candle.animate(new TwinkleAnimation(30, 70, 60, 0, 0.4, Ledcount, TwinklePercent.Percent6));
    // m_candle.animate(new LarsonAnimation(101, 15, 140, 0, 0.5, Ledcount, BounceMode.Front, 15));
    // m_candle.animate(new LarsonAnimation(255, 255, 0, 0, 0.5, Ledcount, BounceMode.Front, 15));
    // m_candle.animate(    new ColorFlowAnimation(255, 255, 0, 0, 0.85, Ledcount, Direction.Forward));
    // m_candle.animate(null);
    resetLights();

    // m_lightValues.addNumber("Light Output", () -> getCurrentLights());


  }
  public void partyMode(){
    m_candle.animate(new RainbowAnimation(1, 1, Ledcount));
  }
  public void epilepsy() {
    m_candle.animate(new StrobeAnimation(255, 255, 255, 0, 98.0 / 256.0, Ledcount));
  }
  public void setDisabledColor() {
    m_candle.setLEDs(0,0,0);
  }
  public void askForCube(){
    resetLights();
    m_candle.animate(    new ColorFlowAnimation(101, 15, 140, 0, 0.85, Ledcount, Direction.Forward));

  }
  public void askForCone(){
    resetLights();
    m_candle.animate(    new ColorFlowAnimation(255, 255, 0, 0, 0.85, Ledcount, Direction.Forward));

  }
  public void resetLights() {
    m_candle.setLEDs(0,0,0);
  }

  public void setCube() {
    // resetLights();
    // m_candle.animate(new LarsonAnimation(101, 15, 140, 0, 0.15, Ledcount, BounceMode.Front, 10));
    m_candle.animate(null);
    m_candle.setLEDs(101,15,140, 0, 0, Ledcount);
  }
  public void setCubeTwinkle() {
    m_candle.animate(new LarsonAnimation(101, 15, 140, 0, 0.5, Ledcount, BounceMode.Front, 15));
  }
  public void setDefault(){
    // resetLights();
    // m_candle.animate(new LarsonAnimation(255, 255, 255, 0, 0.5, Ledcount, BounceMode.Front, 3));

    m_candle.animate(new LarsonAnimation(225, 0, 0, 0, 0.05, Ledcount, BounceMode.Front, 15));

  }

  public void setCone() {
    m_candle.animate(null);
    // resetLights();
    // m_candle.animate(new LarsonAnimation(255, 255, 0, 0, 0.15, Ledcount, BounceMode.Front, 10));
    m_candle.setLEDs(255,255,0, 0,0,Ledcount);
  }
  public void setConeTwinkle() {
    m_candle.animate(new LarsonAnimation(255, 255, 0, 0, 0.5, Ledcount, BounceMode.Front, 15));
  }
  public void setGiven(int RED, int GREEN, int BLUE) {
    m_candle.setLEDs(RED, GREEN, BLUE);
  }


  public double getCurrentLights() {
    return m_candle.configGetParameter(null, kPhoenixDriverPort);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (count == 500){
      if (lCount >30){
        lCount=1;
      }
      m_candle.setLEDs(0,0,0, 0,lCount,30);
      m_candle.setLEDs(255,255,255, 0,0,lCount);

      count = 0;
    }
    count = count+1;
  }

  public void resetAnim() {
  m_candle.animate(null);
  }
}
