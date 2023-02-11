// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
  //arm motors

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
    private RelativeEncoder m_firstLiftMotorEncoder;

    DigitalInput m_limitSwitch = new DigitalInput(Pivot.SWITCH_PORT);

  //set extentions

    private double extendSetPoint = 0;
    private double pivotSetPoint = 0;

  /** Creates a new ExtensionSubsystem. */
  public ArmSubsystem() {
    m_extendMotor.restoreFactoryDefaults();
    m_firstLiftMotor.restoreFactoryDefaults();
    m_secondLiftMotor.restoreFactoryDefaults();

    m_secondLiftMotor.follow(m_firstLiftMotor);

    //encoders
    m_extendMotorEncoder = m_extendMotor.getEncoder();
    m_firstLiftMotorEncoder = m_firstLiftMotor.getEncoder();
 
    //soft limits on motors
    m_firstLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, Pivot.PIVOT_REVERSE_SOFT_LIMIT);
    m_secondLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, Pivot.PIVOT_REVERSE_SOFT_LIMIT);
    m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, Extend.EXTEND_REVERSE_SOFT_LIMIT);

    m_firstLiftMotor.setSoftLimit(SoftLimitDirection.kForward, Pivot.PIVOT_FORWARD_SOFT_LIMIT);
    m_secondLiftMotor.setSoftLimit(SoftLimitDirection.kForward, Pivot.PIVOT_FORWARD_SOFT_LIMIT);
    m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, Extend.EXTEND_FORWARD_SOFT_LIMIT);
  }

  @Override
  public void periodic() {

    if (m_limitSwitch.get()) {
      m_firstLiftMotorEncoder.setPosition(Pivot.MIN_ANGLE);
    }
      
    m_extendPIDController.setGoal(extendSetPoint + extendCorrect(pivotSetPoint));
    m_pivotPIDController.setGoal(pivotSetPoint);
    
    m_firstLiftMotor.set(m_pivotPIDController.calculate(m_firstLiftMotorEncoder.getPosition()));  
    m_extendMotor.set(m_extendPIDController.calculate(m_extendMotorEncoder.getPosition())); 
  }

  public double extendCorrect(double armPosition){
    return 0;
  }

  public double getExtendSetPoint(){
    return extendSetPoint;
  }

  public void setExtendSetPoint(double value){
    extendSetPoint = value;
  }

  public double getPivotSetPoint(){
    return pivotSetPoint;
  }
  public void setPivotSetPoint(double value){
    extendSetPoint = value;
  }
}
