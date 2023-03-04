// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.constants.RobotConstants.Arm.*;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BooleanChecker;

public class ArmSubsystem extends SubsystemBase {
  private Servo m_extendServo = new Servo(Extend.SERVO_PORT); 
  
  private CANSparkMax m_extendMotor = new CANSparkMax(Extend.EXTEND_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_masterPivotMotor = new CANSparkMax(Pivot.MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slavePivotMotor = new CANSparkMax(Pivot.SLAVE_MOTOR_ID, MotorType.kBrushless);

  private PIDController m_pivotPIDController = new PIDController(
      Pivot.P, Pivot.I, Pivot.D);
  private PIDController m_extendPIDController = new PIDController(
      Extend.P, Extend.I, Extend.D);

  private RelativeEncoder m_extendEncoder;
  private WPI_CANCoder m_pivotEncoder;

  // Limit switch on extend for when the arm is fully retracted
  // private DigitalInput m_extendSwitch = new DigitalInput(Pivot.SWITCH_PORT);

  private double m_extendSetpoint = 0;
  private double m_pivotSetpoint = 0;

  // stuff for ratchet
  private double m_targetTime = 0;
  private double m_currentTime = 0;
  private boolean m_ratchetEngaged = true;
  private BooleanChecker m_upChecker = new BooleanChecker(
      () -> m_extendSetpoint > m_extendEncoder.getPosition() + Extend.START_EXTENDING);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_extendMotor.restoreFactoryDefaults();
    m_masterPivotMotor.restoreFactoryDefaults();
    m_slavePivotMotor.restoreFactoryDefaults();

    m_masterPivotMotor.setInverted(true);
    m_slavePivotMotor.setInverted(true);

    m_slavePivotMotor.follow(m_masterPivotMotor);

    // encoders
    m_extendEncoder = m_extendMotor.getEncoder();
    CANCoderConfiguration config = new CANCoderConfiguration();
    m_pivotEncoder = new WPI_CANCoder(Pivot.PIVOT_CANCODER_ID);
    config.sensorCoefficient = Pivot.CANCODER_COEFFICIENT;
    config.unitString = ("rotations");
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    // counterclockwise is positive when facing LED on CANCoder
    config.sensorDirection = false; 
    config.magnetOffsetDegrees = Pivot.CANCODER_OFFSET_DEGREES;
    m_pivotEncoder.configAllSettings(config);

    m_pivotEncoder.setPosition(0);
    m_extendEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // Give the pivot motor voltage
    m_pivotPIDController.setSetpoint(m_pivotSetpoint);
    m_extendPIDController.setSetpoint(m_extendSetpoint);

    m_masterPivotMotor.set(MathUtil.clamp(m_pivotPIDController.calculate(m_pivotEncoder.getPosition()),
        -Pivot.MAX_VOLTAGE, Pivot.MAX_VOLTAGE));

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

  public double getExtendSetpoint(){
    return m_extendSetpoint;
  }

  public void setExtendSetpoint(double value){
    m_extendSetpoint = value;
  }

  public double getPivotPosition() {
    return m_pivotEncoder.getPosition();
  }

  public double getPivotSetpoint(){
    return m_pivotSetpoint;
  }

  public void setPivotSetpoint(double value){
    m_pivotSetpoint = value;
  }

  public void resetEncoders() {
    m_extendEncoder.setPosition(Extend.FULLY_EXTENDED);
    m_extendPIDController.setSetpoint(Extend.FULLY_EXTENDED);
    m_extendSetpoint = Extend.FULLY_EXTENDED;

    m_pivotEncoder.setPosition(0);
    m_pivotPIDController.setSetpoint(0);
    m_pivotSetpoint = 0;
  }
  // Arm pivot is at an angle where 0 should be
  public void printCANCoderOffset() {
    System.out.println(-m_pivotEncoder.getAbsolutePosition());
  }
}
