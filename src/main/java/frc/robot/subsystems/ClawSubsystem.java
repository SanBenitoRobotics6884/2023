// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.constants.RobotConstants.Claw.*;
import static frc.robot.constants.RobotConstants.Feedback.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  CANSparkMax m_motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_motor.getEncoder();
  Debouncer m_rumble = new Debouncer(0.2, DebounceType.kRising);

  double m_rotations = 0;
  double hue = 0;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
  SparkMaxPIDController m_pidController = m_motor.getPIDController();

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_pidController.setP(P);
    m_pidController.setI(I);
    m_pidController.setD(D);
    m_pidController.setOutputRange(-MAX_VOLTAGE, MAX_VOLTAGE);
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    m_pidController.setReference(m_rotations, ControlType.kPosition);
    SmartDashboard.putNumber("claw encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("setpoint", m_rotations);
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

  /** Return when to rumble */
  public boolean getStatus() {
    return m_rumble.calculate(
        m_encoder.getVelocity() <= CLAW_RUMBLE_MAX_VELOCITY 
        && (m_encoder.getPosition() - m_rotations) * 3 > CLAW_RUMBLE_MIN_VOLTAGE);
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