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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.Arm.Pivot;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private CANSparkMax m_masterPivotMotor = new CANSparkMax(Pivot.MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slavePivotMotor = new CANSparkMax(Pivot.SLAVE_MOTOR_ID, MotorType.kBrushless);

  private PIDController m_pivotPIDController = new PIDController(
      Pivot.P, Pivot.I, Pivot.D);
  private WPI_CANCoder m_pivotEncoder;
  private double m_pivotSetpoint = 0;

  public PivotSubsystem() {
    m_masterPivotMotor.restoreFactoryDefaults(); // setting up motors
    m_slavePivotMotor.restoreFactoryDefaults();

    m_masterPivotMotor.setInverted(true);
    m_slavePivotMotor.setInverted(true);
    m_slavePivotMotor.follow(m_masterPivotMotor);
    
    CANCoderConfiguration config = new CANCoderConfiguration(); // encoders
    m_pivotEncoder = new WPI_CANCoder(Pivot.PIVOT_CANCODER_ID);
    config.unitString = ("rotations");
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    
    config.sensorDirection = false;
    config.magnetOffsetDegrees = Pivot.CANCODER_OFFSET_DEGREES;
    m_pivotEncoder.configAllSettings(config);

    m_pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Give the pivot motor voltage
    m_pivotPIDController.setSetpoint(m_pivotSetpoint);
    m_masterPivotMotor.set(MathUtil.clamp(m_pivotPIDController.calculate(m_pivotEncoder.getPosition()),
      -Pivot.MAX_VOLTAGE, Pivot.MAX_VOLTAGE));
  }

  public double getPivotPosition() {
    return m_pivotEncoder.getPosition();
  }

  public double getPivotSetpoint() {
    return m_pivotSetpoint;
  }

  public void setPivotSetpoint(double value){
    m_pivotSetpoint = value;
  }

  public void printCANCoderOffset() {
    System.out.println(-m_pivotEncoder.getAbsolutePosition());
  }
}
