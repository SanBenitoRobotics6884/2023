// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.RobotConstants.Pivot.*;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private CANSparkMax m_masterMotor = new CANSparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slaveMotor = new CANSparkMax(SLAVE_MOTOR_ID, MotorType.kBrushless);

  private PIDController m_pid = new PIDController(P, I, D);
  private WPI_CANCoder m_encoder;
  private double m_setpoint = 0;

  public PivotSubsystem() {
    m_masterMotor.restoreFactoryDefaults(); // setting up motors
    m_slaveMotor.restoreFactoryDefaults();

    m_masterMotor.setInverted(true);
    m_slaveMotor.setInverted(true);
    m_slaveMotor.follow(m_masterMotor);
    
    CANCoderConfiguration config = new CANCoderConfiguration(); // encoders
    m_encoder = new WPI_CANCoder(PIVOT_CANCODER_ID);
    config.unitString = ("rotations");
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    
    config.sensorDirection = false;
    config.magnetOffsetDegrees = CANCODER_OFFSET_DEGREES;
    m_encoder.configAllSettings(config);

    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Give the pivot motor voltage
    m_pid.setSetpoint(m_setpoint);
    m_masterMotor.set(MathUtil.clamp(m_pid.calculate(m_encoder.getPosition()),
      -MAX_VOLTAGE, MAX_VOLTAGE));
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getSetpoint() {
    return m_setpoint;
  }

  public void setSetpoint(double value){
    m_setpoint = value;
  }

  public void printCANCoderOffset() {
    System.out.println(-m_encoder.getAbsolutePosition());
  }

  public CommandBase getDownCommand() {
    return runOnce(() -> setSetpoint(HYBRID_SETPOINT));
  }

  public CommandBase getPickUpCommand() {
    return runOnce(() -> setSetpoint(MID_SETPOINT));
  }

  public CommandBase getPlaceCommand() {
    return runOnce(() -> setSetpoint(HIGH_SETPOINT));
  }
}
