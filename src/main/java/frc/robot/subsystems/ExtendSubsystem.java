// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.RobotConstants.Extend.*;
import frc.robot.util.BooleanChecker;

public class ExtendSubsystem extends SubsystemBase {
  private Servo m_servo = new Servo(SERVO_PORT); 
  private CANSparkMax m_motor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new ExtendSubsystem. */
  private PIDController m_pid = new PIDController(P, I, D);
  private RelativeEncoder m_encoder;
  private double m_setpoint = 0;

  // stuff for ratchet
  private double m_targetTime = 0;
  private double m_currentTime = 0;
  private boolean m_ratchetEngaged = true;
  private BooleanChecker m_upChecker = new BooleanChecker( 
    () -> m_setpoint > m_encoder.getPosition() + START_EXTENDING);
    
  public ExtendSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    m_pid.setSetpoint(m_setpoint);
    double extendOutput;
    if (m_upChecker.check() && m_ratchetEngaged) {
      m_targetTime = Timer.getFPGATimestamp() + SERVO_DELAY;
      m_currentTime = Timer.getFPGATimestamp();
      m_ratchetEngaged = false;
    }
    if (Timer.getFPGATimestamp() > m_targetTime) {
      if (m_encoder.getPosition() > m_setpoint - SETPOINT_TOLERANCE) {
        m_ratchetEngaged = true;
      }

      if (m_ratchetEngaged) {
        extendOutput = MathUtil.clamp(
          m_pid.calculate(m_encoder.getPosition()),
          -MAX_VOLTAGE_RETRACT, 0);

      } else {
        extendOutput = MathUtil.clamp(
          m_pid.calculate(m_encoder.getPosition()),
          -MAX_VOLTAGE_RETRACT, MAX_VOLTAGE_EXTEND);
      }
    } else { 
      if (Timer.getFPGATimestamp() - m_currentTime < BACK_TIME) {
        extendOutput = -BACK_VOLTAGE;
      } else {
        extendOutput = 0;
      }
    }
    m_motor.set(extendOutput);
    if (m_ratchetEngaged) {
      m_servo.set(RATCHET_ENGAGED);
    } else {
      m_servo.set(RATCHET_DISENGAGED);
    }
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public void setSetpoint(double value) {
    m_setpoint = value;
  }
  
  public CommandBase getRetractCommand() {
    return runOnce(() -> setSetpoint(HYBRID_SETPOINT));
  }

  public CommandBase getMidCommand() {
    return runOnce(() -> setSetpoint(MID_SETPOINT));
  }

  public CommandBase getExtendCommand() {
    return runOnce(() -> setSetpoint(HIGH_SETPOINT));
  }
}

