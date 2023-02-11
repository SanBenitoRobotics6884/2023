// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Turret.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  CANSparkMax m_turretMotor = new CANSparkMax(TURRET_ID, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_turretMotor.getEncoder();
  DigitalInput m_leftSwitch = new DigitalInput(LEFT_SWITCH_PORT);
  DigitalInput m_rightSwitch = new DigitalInput(RIGHT_SWITCH_PORT);
  ProfiledPIDController m_turretPID = new ProfiledPIDController(
    P, I, D,
    new TrapezoidProfile.Constraints(MAX_VELOCITY * GEAR_RATIO, MAX_ACCELERATION * GEAR_RATIO)
  );

  /** Creates a new TurretSubsystemAlt2. */
  public TurretSubsystem() {
    m_turretMotor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    double measurement = m_encoder.getPosition();
    double output = m_turretPID.calculate(measurement);

    /**
    SmartDashboard.putNumber("Turret Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Turret Encoder Velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("Turret Setpoint", m_turretPID.getGoal().position);
    SmartDashboard.putNumber("ProfiledPIDController output", output);
    */

    if (giveNormalOutput(measurement, output > 0)) {
      // Give the turret the output from the profiled PID if within soft limits or moving towards them
      m_turretMotor.setVoltage(output);
    } else if (giveScaledOutput(measurement, output > 0)) {
      // Give the turret scaled down output if outside soft limits and moving towards hard limits
      // Might make more complicated logic to ensure slow turret near bounds later
      m_turretMotor.setVoltage(BOUNDARY_SCALE * output);
    } else {
      // Set the encoder accordingly if limit switch is hit and then go to 0
      if (m_leftSwitch.get()) {
        m_encoder.setPosition(LEFT_SWITCH_ANGLE);
      } else if (m_rightSwitch.get()) {
        m_encoder.setPosition(RIGHT_SWITCH_ANGLE);
      }
      m_turretMotor.setVoltage(0);
      m_turretPID.setGoal(0);
    }
  }

  /** Set the encoder value to 0 */
  public void reset() {
    m_encoder.setPosition(0);
    m_turretPID.setGoal(0);
  }

  /** Set the desired value of the encoder */
  public void setSetpoint(double setpoint) {
    m_turretPID.setGoal(setpoint);
  }

  /** Get the desired value of the encoder */
  public double getSetpoint() {
    return m_turretPID.getGoal().position;
  }

  public void turnLeft() {
    double newSetpoint = getSetpoint() - MANUAL_ROTATE;
    if (newSetpoint >= HARD_MIN_ANGLE) {
      setSetpoint(newSetpoint);
    }
  }

  public void turnRight() {
    double newSetpoint = getSetpoint() + MANUAL_ROTATE;
    if (newSetpoint <= HARD_MAX_ANGLE) {
      setSetpoint(newSetpoint);
    }
  }

  /** Return when to rumble the left side of the controller */
  public boolean getLeftStatus() {
    return m_encoder.getPosition() < LEFT_RUMBLE_ANGLE;
  }

  /** Return when to rumble the right side of the controller */
  public boolean getRightStatus() {
    return m_encoder.getPosition() > RIGHT_RUMBLE_ANGLE;
  }

  /** Return the motor temperature for when testing */
  public double getMotorTemperature() {
    return m_turretMotor.getMotorTemperature();
  }

  private boolean giveNormalOutput(double measurement, boolean movingRight) {
    /*
    return 
      ((SOFT_MIN_ANGLE <= measurement && measurement <= SOFT_MAX_ANGLE)
      || (HARD_MIN_ANGLE <= measurement && measurement < SOFT_MIN_ANGLE && movingRight)
      || (HARD_MAX_ANGLE >= measurement && measurement > SOFT_MAX_ANGLE && !movingRight));
    */
    return
      ((SOFT_MIN_ANGLE <= measurement && measurement <= SOFT_MAX_ANGLE)
      || (HARD_MIN_ANGLE <= measurement && measurement < SOFT_MIN_ANGLE && movingRight)
      || (HARD_MAX_ANGLE >= measurement && measurement > SOFT_MAX_ANGLE && !movingRight))
      && (!m_leftSwitch.get() && !m_rightSwitch.get());
  }

  private boolean giveScaledOutput(double measurement, boolean movingRight) {
    /*
    return
      ((HARD_MIN_ANGLE <= measurement && measurement < SOFT_MIN_ANGLE && !movingRight)
      || (HARD_MAX_ANGLE >= measurement && measurement > SOFT_MAX_ANGLE && movingRight))
    */
    return !m_leftSwitch.get() && !m_rightSwitch.get();
  }
}
