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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.Arm.Extend;
import frc.robot.util.BooleanChecker;

public class ExtendSubsystem extends SubsystemBase {
  private Servo m_extendServo = new Servo(Extend.SERVO_PORT); 
  private CANSparkMax m_extendMotor = new CANSparkMax(Extend.EXTEND_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new ExtendSubsystem. */
  private PIDController m_extendPIDController = new PIDController(
    Extend.P, Extend.I, Extend.D);
  private RelativeEncoder m_extendEncoder;
  private double m_extendSetpoint = 0;

  // stuff for ratchet
  private double m_targetTime = 0;
  private double m_currentTime = 0;
  private boolean m_ratchetEngaged = true;
  private BooleanChecker m_upChecker = new BooleanChecker( 
    () -> m_extendSetpoint > m_extendEncoder.getPosition() + Extend.START_EXTENDING);
    
  public ExtendSubsystem() {
    m_extendMotor.restoreFactoryDefaults();
    m_extendEncoder = m_extendMotor.getEncoder();
    m_extendEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    m_extendPIDController.setSetpoint(m_extendSetpoint);
    double extendOutput;
    if (m_upChecker.check() && m_ratchetEngaged) {
      m_targetTime = Timer.getFPGATimestamp() + Extend.SERVO_DELAY;
      m_currentTime = Timer.getFPGATimestamp();
      m_ratchetEngaged = false;
    }
    if (Timer.getFPGATimestamp() > m_targetTime) {
      if (m_extendEncoder.getPosition() > m_extendSetpoint - Extend.SETPOINT_TOLERANCE) {
        m_ratchetEngaged = true;
      }

      if (m_ratchetEngaged) {
        extendOutput = MathUtil.clamp(
          m_extendPIDController.calculate(m_extendEncoder.getPosition()),
          -Extend.MAX_VOLTAGE_RETRACT, 0);

      } else {
        extendOutput = MathUtil.clamp(
          m_extendPIDController.calculate(m_extendEncoder.getPosition()),
          -Extend.MAX_VOLTAGE_RETRACT, Extend.MAX_VOLTAGE_EXTEND);
      }
    } else { 
      if (Timer.getFPGATimestamp() - m_currentTime < Extend.BACK_TIME) {
        extendOutput = -Extend.BACK_VOLTAGE;
      } else {
        extendOutput = 0;
      }
    }
    m_extendMotor.set(extendOutput);
    if (m_ratchetEngaged) {
      m_extendServo.set(Extend.RATCHET_ENGAGED);
    } else {
      m_extendServo.set(Extend.RATCHET_DISENGAGED);
    }
  }

   public double getExtendPosition() {
    return m_extendEncoder.getPosition();
   }

   public double getExtendSetpoint() {
    return m_extendSetpoint;
   }

   public void setExtendSetpoint(double value) {
    m_extendSetpoint = value;
   }
   


  
  }

