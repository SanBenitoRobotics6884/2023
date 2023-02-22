// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection; // Is commented out
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.BooleanChecker;

import static frc.robot.ConstantsFolder.RobotConstants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
  // arm motors
  private Servo m_extendServo = new Servo(Extend.SERVO_PORT); 
  
  private CANSparkMax m_extendMotor = new CANSparkMax(Extend.EXTEND_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_firstLiftMotor = new CANSparkMax(Pivot.LEFT_MOTOR, MotorType.kBrushless);
  private CANSparkMax m_secondLiftMotor = new CANSparkMax(Pivot.RIGHT_MOTOR, MotorType.kBrushless);

  // Profiled PID for the Extend and Pivoting Motors
  private ProfiledPIDController m_pivotPIDController = new ProfiledPIDController(
      Pivot.P, Pivot.I, Pivot.D,
      new TrapezoidProfile.Constraints(Pivot.GEAR_RATIO * Pivot.MAX_VELOCITY, Pivot.GEAR_RATIO * Pivot.MAX_ACCELERATION));
  private ProfiledPIDController m_extendPIDController = new ProfiledPIDController(
      Extend.P, Extend.I, Extend.D,
      new TrapezoidProfile.Constraints(Extend.GEAR_RATIO * Extend.MAX_VELOCITY, Extend.GEAR_RATIO * Extend.MAX_ACCELERATION));

  private RelativeEncoder m_extendMotorEncoder;
  private WPI_CANCoder m_pivotMotorEncoder;

    DigitalInput m_limitSwitch = new DigitalInput(Pivot.SWITCH_PORT);

  // intialize setpoints
  private double m_extendSetpoint = 0;
  private double m_pivotSetpoint = 0;

  // stuff for ratchet
  private boolean m_waiting = false;
  private double m_timestamp = 0;
  private double m_servoValue = Extend.RATCHET_ENGAGED;
  private BooleanChecker m_upChecker = new BooleanChecker(
      () -> m_extendSetpoint > m_extendMotorEncoder.getPosition() + Extend.SETPOINT_ERROR);

  /** Creates a new ExtensionSubsystem. */
  public ArmSubsystem() {
    m_extendMotor.restoreFactoryDefaults();
    m_firstLiftMotor.restoreFactoryDefaults();
    m_secondLiftMotor.restoreFactoryDefaults();

    m_secondLiftMotor.follow(m_firstLiftMotor);

    // encoders
    m_extendMotorEncoder = m_extendMotor.getEncoder();
    //cancoder and setting the config values
    CANCoderConfiguration config = new CANCoderConfiguration();
    m_pivotMotorEncoder = new WPI_CANCoder(Pivot.PIVOT_CANCODER_ID);
    config.sensorCoefficient = Pivot.CANCODER_COEFFECIENT;
    config.unitString = ("rotations");
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    // counter clockwise is false, clockwise is true, when facing LED on CANCoder
    config.sensorDirection = false;
    config.initializationStrategy = SensorInitializationStrategy.BootToZero;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
    m_pivotMotorEncoder.configAllSettings(config);

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
    // 
    if (m_limitSwitch.get()) {
      m_extendMotorEncoder.setPosition(Extend.FULL_CLOSED);
      m_extendPIDController.setGoal(Extend.FULL_CLOSED);
    }
    
    if (m_upChecker.check()) {
      // When we are starting to try to extend out, we need a wait
      m_waiting = true;
      m_timestamp = Timer.getFPGATimestamp() + Extend.SERVO_DELAY;
      // We wait for the servo to fully disengage
      m_servoValue = Extend.RATCHET_DISENGAGED;
      m_firstLiftMotor.set(0);
      m_extendMotor.set(0);
    } else if (!m_waiting) {
      // While we're going we need to update the goal
      m_extendPIDController.setGoal(m_extendSetpoint);
      m_pivotPIDController.setGoal(m_pivotSetpoint);

      // If the extension is within the error, reengage the ratchet and stop the motor, otherwise give it the pid output
      if (Math.abs(m_extendMotorEncoder.getPosition() - m_extendSetpoint) > Extend.SETPOINT_ERROR) {
        m_extendMotor.set(m_extendPIDController.calculate(m_extendMotorEncoder.getPosition()));
      } else {
        m_extendMotor.set(0);
        m_servoValue = Extend.RATCHET_ENGAGED;
      }
      // If the pivot is within the error, stop the motor, otherwise, give it the pid output
      if (Math.abs(m_pivotMotorEncoder.getPosition() - m_pivotSetpoint) < Pivot.SETPOINT_ERROR) {
        m_firstLiftMotor.set(m_pivotPIDController.calculate(m_pivotMotorEncoder.getPosition()));
      } else {
        m_firstLiftMotor.set(0);
      }
    } else {
      // While waiting, put both motors to false.
      if (Timer.getFPGATimestamp() + 0.020 >= m_timestamp) {
        m_waiting = false;
      }
      m_firstLiftMotor.set(0);
      m_extendMotor.set(0);
    }
    m_extendServo.set(m_servoValue);
  }

  public double getExtendSetpoint(){
    return m_extendSetpoint;
  }

  public void setExtendSetpoint(double value){
    m_extendSetpoint = value;
  }

  public double getPivotSetpoint(){
    return m_pivotSetpoint;
  }

  public void setPivotSetpoint(double value){
    m_extendSetpoint = value;
  }
}
