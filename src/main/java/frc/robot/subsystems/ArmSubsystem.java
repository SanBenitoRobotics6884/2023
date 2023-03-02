// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.BooleanChecker;

import static frc.robot.ConstantsFolder.RobotConstants.Arm.*;

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
  private DigitalInput m_extendSwitch = new DigitalInput(Pivot.SWITCH_PORT);

  private double m_extendSetpoint = 0;
  private double m_pivotSetpoint = 0;

  // stuff for ratchet
  private boolean m_waiting = false;
  private double m_timestamp = 0;
  private double m_servoValue = Extend.RATCHET_ENGAGED;
  private BooleanChecker m_upChecker = new BooleanChecker(
      () -> m_extendSetpoint > m_extendEncoder.getPosition() + Extend.SETPOINT_ERROR);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_extendMotor.restoreFactoryDefaults();
    m_masterPivotMotor.restoreFactoryDefaults();
    m_slavePivotMotor.restoreFactoryDefaults();

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

    // soft limits on motors
    // Leaving it commented out until we understand it better
    /*
    m_firstLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, Pivot.PIVOT_REVERSE_SOFT_LIMIT);
    m_secondLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, Pivot.PIVOT_REVERSE_SOFT_LIMIT);
    m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, Extend.EXTEND_REVERSE_SOFT_LIMIT);

    m_firstLiftMotor.setSoftLimit(SoftLimitDirection.kForward, Pivot.PIVOT_FORWARD_SOFT_LIMIT);
    m_secondLiftMotor.setSoftLimit(SoftLimitDirection.kForward, Pivot.PIVOT_FORWARD_SOFT_LIMIT);
    m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, Extend.EXTEND_FORWARD_SOFT_LIMIT);
    */
  }

  @Override
  public void periodic() {
    // If the limit switch for the arm is hit, set the encoders
    if (m_extendSwitch.get()) {
      m_extendEncoder.setPosition(Extend.FULLY_RETRACTED);
      m_extendPIDController.setSetpoint(Extend.FULLY_RETRACTED);
      m_extendSetpoint = Extend.FULLY_RETRACTED;
    }

    // Give the pivot motor voltage
    m_pivotPIDController.setSetpoint(m_pivotSetpoint);
    m_masterPivotMotor.set(MathUtil.clamp(m_pivotPIDController.calculate(m_pivotEncoder.getPosition()),
        -Pivot.MAX_VOLTAGE, Pivot.MAX_VOLTAGE));
    if (m_upChecker.check()) {
      // When we are starting to try to extend out, we need a wait
      m_waiting = true;
      // The time we stop waiting
      m_timestamp = Timer.getFPGATimestamp() + Extend.SERVO_DELAY;
      // We wait for the servo to fully disengage
      m_servoValue = Extend.RATCHET_DISENGAGED;
      m_extendMotor.set(0);
    } else if (!m_waiting) {
      // While we're going (not waiting) we need to update the setpoint
      m_extendPIDController.setSetpoint(m_extendSetpoint);
      // If the extension is within the error, reengage the ratchet and stop the motor, otherwise give it the pid output
      if (Math.abs(m_extendEncoder.getPosition() - m_extendSetpoint) > Extend.SETPOINT_ERROR) {
        m_extendMotor.set(MathUtil.clamp(m_extendPIDController.calculate(m_extendEncoder.getPosition()),
            -Extend.MAX_VOLTAGE, Extend.MAX_VOLTAGE));
      } else {
        m_extendMotor.set(0);
        m_servoValue = Extend.RATCHET_ENGAGED;
      }
    } else {
      // While waiting, don't give extend motor voltage
      if (Timer.getFPGATimestamp() + 0.020 >= m_timestamp) {
        m_waiting = false;
      }
      // Since the spring pushes out against the ratchet, the extension shouldn't move until after
      // the ratchet is disengaged, and by that time the spring is pushing it out. 
      m_extendMotor.set(0);
    }
    m_extendServo.set(m_servoValue);
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