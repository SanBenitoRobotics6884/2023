// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import static frc.robot.RobotConstants.Claw.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  CANSparkMax m_motor = new CANSparkMax(0, MotorType.kBrushless);
 
  double m_rotations = 0;
  double m_CurrentButton = 0;
  double hue = 0;
  double prevHue = 0;
  boolean isClawClosed = false;
  boolean triggerState = false;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  public SparkMaxPIDController m_pidController = m_motor.getPIDController();
  Joystick m_joystick = new Joystick(0);

  public ClawSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kFF);
    m_pidController.setIZone(kIZone); 
  }

  @Override
  public void periodic() {
    setReference(m_rotations);
  }

   
  public void setRotations(double rotations) {
    m_rotations = rotations;
  }

  public double getRotations() {
    return m_rotations;
  }
  

  public void setTriggerState(boolean state) {
    triggerState = state;
  }

  public boolean getTriggerState() {
    return triggerState;
  }

  public void move(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  public double getCurrentButton() {
    return m_CurrentButton;
  }
 
  public void setReference(double rotations) {
    if (!triggerState) {  
      if (isClawClosed = false) {
        isClawClosed = true;
        move(rotations);
      } else {
        isClawClosed = false;
        move(kOpen);
      }
    }
  }

  public void colorCheck() {
    hue = getHue(m_colorSensor.getRawColor());
    
      if (kConeMinHue <= hue && hue <= kConeMaxHue) {
        m_rotations = kConeClose;
        setReference(m_rotations);
      } else if (kCubeMinHue <= hue && hue <= kCubeMaxHue) {
        m_rotations = kCubeClose;
        setReference(m_rotations);
      }
  }


  public double getHue (RawColor color) {
    double Red = Double.valueOf(color.red);
    double Blue = Double.valueOf(color.blue);
    double Green = Double.valueOf(color.green);
    double preHue = 0;
    
    
    if (Red == Math.max(Red, Math.max(Green, Blue))) {
      preHue = (Green - Blue) / (Red - Math.min(Green, Blue));
    } else if (Green == Math.max(Red, Math.max(Green, Blue))) {
      preHue = 2 + (Red - Blue) / (Green - Math.min(Red, Blue));
    } else {
      preHue = 4 + (Red - Green) / (Blue - Math.min(Green, Red));
    }

    if (preHue < 0) {preHue += 6;}
    return preHue * 60;
  }
}