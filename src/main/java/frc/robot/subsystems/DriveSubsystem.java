// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drive.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_FlMotor;
  private CANSparkMax m_FRMotor;
  private CANSparkMax m_BRMotor;
  private CANSparkMax m_BLMotor;
  private MotorControllerGroup rControllerGroup;
  private MotorControllerGroup lControllerGroup;
  private DifferentialDrive drive;
  RelativeEncoder m_rightEncoder;
  RelativeEncoder m_leftEncoder;
  
  
  WPI_Pigeon2 gyro;

  ChassisSpeeds chassisSpeeds;
  public DriveSubsystem() {
    m_FlMotor = new CANSparkMax(FL_ID, MotorType.kBrushless);
    m_FRMotor = new CANSparkMax(FR_ID, MotorType.kBrushless);
    m_BRMotor = new CANSparkMax(BL_ID, MotorType.kBrushless);
    m_BLMotor = new CANSparkMax(BR_ID, MotorType.kBrushless);
    

    
    rControllerGroup = new MotorControllerGroup(m_FRMotor, m_BRMotor);
    lControllerGroup = new MotorControllerGroup(m_FlMotor, m_BLMotor);

    rControllerGroup.setInverted(true);
    lControllerGroup.setInverted(false);

    drive = new DifferentialDrive(rControllerGroup, lControllerGroup);
    
    m_rightEncoder = m_FRMotor.getAlternateEncoder(
      SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
      
     m_leftEncoder = m_FRMotor.getAlternateEncoder(
        SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    m_rightEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_leftEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    m_leftEncoder.setVelocityConversionFactor(BL_ID);
    
    gyro = new WPI_Pigeon2(PIGEON_ID);

    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch= MOUNT_PITCH;
    config.MountPoseRoll= MOUNT_ROLL;
    config.MountPoseYaw = MOUNT_YAW;
    
    
    gyro.configAllSettings(config);
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void NormalDrive(Double foward, Double rotation){
    drive.arcadeDrive(foward*NORMAL_FF, rotation*NORMAL_FF);

  }
  
  public void TurboJoystickDrive(Double foward, Double rotation){
    drive.arcadeDrive(foward*TURBO_FF, rotation*TURBO_FF);

  }
  public void ResetEncoder(){
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }
  
  public Rotation2d getRotation2D(){
    return gyro.getRotation2d();
  }
  
  public double getLeftDistance(){
    return m_leftEncoder.getPosition();
  }

  public double getRightDistance(){
    return m_rightEncoder.getPosition();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getVelocity();
  }
  public void SetMotorVoltage(double rightVoltage, double leftVoltage){
    rControllerGroup.setVoltage(rightVoltage);
    lControllerGroup.setVoltage(leftVoltage);
  }
  


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
