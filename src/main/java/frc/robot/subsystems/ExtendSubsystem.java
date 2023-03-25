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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.RobotConstants.Extend.*;
import frc.robot.util.BooleanChecker;

public class ExtendSubsystem extends SubsystemBase {
  private CANSparkMax m_motor = new CANSparkMax(EXTEND_MOTOR_ID, MotorType.kBrushless);
  private PIDController m_pid = new PIDController(P, I, D);
  private RelativeEncoder m_encoder;
  private double m_setpoint = 0;

  // This servo engages and disengages a ratchet that stops the extension from going out
  private Servo m_servo = new Servo(SERVO_PORT); 
  // These variables tell the extend motor when it is okay to extend out
  private double m_targetTime = 0;
  private double m_currentTime = 0;
  private boolean m_ratchetEngaged = true;
  // Don't start extending out until this boolean supplier rises from false to true
  private BooleanChecker m_upChecker = new BooleanChecker( 
      () -> m_setpoint > m_encoder.getPosition() + START_EXTENDING);
  
  /** Creates a new ExtendSubsystem. */
  public ExtendSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    m_pid.setSetpoint(m_setpoint);
    if (m_upChecker.check() && m_ratchetEngaged) {
      // Start a timer to wait for the servo to disengage the ratchet when trying to extend out
      m_targetTime = Timer.getFPGATimestamp() + SERVO_DELAY;
      m_currentTime = Timer.getFPGATimestamp();
      m_ratchetEngaged = false;
    }

    // This block decides what to output to the motor, trying to get the extension to the setpoint
    double output;
    if (Timer.getFPGATimestamp() > m_targetTime) {
      // While we are past the wait, give the motor voltage based on the pid output
      // If we are within SETPOINT_TOLERANCE before we reach the setpoint, engage the ratchet so
      // it won't go farther
      if (m_encoder.getPosition() > m_setpoint - SETPOINT_TOLERANCE) {
        m_ratchetEngaged = true;
      }

      // Don't give motor voltage to extend out when ratchet is engaged, otherwise it is fine
      if (m_ratchetEngaged) {
        output = MathUtil.clamp(
          m_pid.calculate(m_encoder.getPosition()),
          -MAX_VOLTAGE_RETRACT, 0);
      } else {
        output = MathUtil.clamp(
          m_pid.calculate(m_encoder.getPosition()),
          -MAX_VOLTAGE_RETRACT, MAX_VOLTAGE_EXTEND);
      }
    } else { 
      // During the wait, go briefly go back a little to make sure the ratchet doesn't get stuck
      // This was Connor's idea and the extension has been reliable ever since.
      if (Timer.getFPGATimestamp() - m_currentTime < BACK_TIME) {
        output = -BACK_VOLTAGE;
      } else {
        output = 0;
      }
    }

    // Set the motor to the decided output and set the servo according to whether the raatchet
    // should be engaged or not
    m_motor.set(output);
    if (m_ratchetEngaged) {
      m_servo.set(RATCHET_ENGAGED);
    } else {
      m_servo.set(RATCHET_DISENGAGED);
    }

    SmartDashboard.putNumber("extend output", output);
    SmartDashboard.putNumber("extend setpoint", getSetpoint());
    SmartDashboard.putNumber("extend position", getPosition());
  }

  /** @returns How far arm has extended from encoder */
  public double getPosition() {
    return m_encoder.getPosition();
  }

  /** @returns The desired position of the extension */
  public double getSetpoint() {
    return m_setpoint;
  }

  /** Set the desired position of the extension */
  public void setSetpoint(double value) {
    m_setpoint = value;
  }
  
  /** @returns A command that sets the desired position to be retracted */
  public Command getRetractCommand() {
    return runOnce(() -> setSetpoint(HYBRID_SETPOINT));
  }

  /** @returns A command that sets the desired position to be in the middle*/
  public Command getMidCommand() {
    return runOnce(() -> setSetpoint(MID_SETPOINT));
  }

  /** @returns A command that sets the desired position to be extended */
  public Command getExtendCommand() {
    return runOnce(() -> setSetpoint(HIGH_SETPOINT));
  }
}

