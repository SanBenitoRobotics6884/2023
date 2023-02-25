// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.ConstantsFolder.RobotConstants.Claw.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  CANSparkMax m_motor = new CANSparkMax(0, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_motor.getEncoder();

  double m_rotations = 0;
  double m_CurrentButton = 0;
  double hue = 0;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  PIDController m_pidController = new PIDController(P, I, D);
  Joystick m_joystick = new Joystick(0);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_motor.setInverted(false); // I don't recall whether or not we need it inverted (closing -> +)
    m_motor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    m_motor.set(MathUtil.clamp(m_pidController.calculate(m_encoder.getPosition(), m_rotations), 
        -MAX_VOLTAGE, MAX_VOLTAGE));
  }

   
  public void setRotations(double rotations) {
    m_rotations = rotations;
  }

  public double getRotations() {
    return m_rotations;
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
    m_rotations = 0;
  }

  public void colorCheck() {
    hue = getHue(m_colorSensor.getRawColor());
    
      if (CONE_MIN_HUE <= hue && hue <= CONE_MAX_HUE) {
        m_rotations = CONE_SETPOINT;
      } else if (CUBE_MIN_HUE <= hue && hue <= CUBE_MAX_HUE) {
        m_rotations = CUBE_SETPOINT;
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