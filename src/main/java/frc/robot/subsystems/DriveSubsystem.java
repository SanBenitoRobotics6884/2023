// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Drive.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_FLMotor;
  private CANSparkMax m_FRMotor;
  private CANSparkMax m_BRMotor;
  private CANSparkMax m_BLMotor;

  private MotorControllerGroup m_rControllerGroup;
  private MotorControllerGroup m_lControllerGroup;
  private DifferentialDrive m_drive;

  RelativeEncoder m_rightEncoder;
  RelativeEncoder m_leftEncoder;
  //make sure to output degrees as negative!!!!!!!!!
 // WPI_Pigeon2 m_gyro;
 

  ADIS16470_IMU m_gyro;
  

  ChassisSpeeds chassisSpeeds;
  public DriveSubsystem(ADIS16470_IMU gyro) {
    m_FLMotor = new CANSparkMax(FL_ID, MotorType.kBrushless);
    m_FRMotor = new CANSparkMax(FR_ID, MotorType.kBrushless);
    m_BRMotor = new CANSparkMax(BR_ID, MotorType.kBrushless);
    m_BLMotor = new CANSparkMax(BL_ID, MotorType.kBrushless);
    
    m_rControllerGroup = new MotorControllerGroup(m_FRMotor, m_BRMotor);
    m_lControllerGroup = new MotorControllerGroup(m_FLMotor, m_BLMotor);
  
    m_FLMotor.restoreFactoryDefaults();    
    m_FRMotor.restoreFactoryDefaults();
    m_BRMotor.restoreFactoryDefaults(); 
    m_BLMotor.restoreFactoryDefaults();

    m_rControllerGroup.setInverted(false);
    m_lControllerGroup.setInverted(true);
   

    m_drive = new DifferentialDrive(m_rControllerGroup, m_lControllerGroup);
   
    m_rightEncoder = m_FRMotor.getAlternateEncoder(
        Type.kQuadrature, 8192);
    m_leftEncoder = m_FLMotor.getAlternateEncoder(
        Type.kQuadrature, 8192);

    //m_rightEncoder.setInverted(true);
    //m_leftEncoder.setInverted(false);


    m_rightEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    m_leftEncoder.setPositionConversionFactor(POSITION_CONVERSION);
    
    m_rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    m_leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    
  
    m_gyro = gyro;
    m_gyro.reset();
    
  // m_gyro = new WPI_Pigeon2(PIGEON_ID);
    /* 
     Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch= MOUNT_PITCH;
    config.MountPoseRoll= MOUNT_ROLL;
    config.MountPoseYaw = MOUNT_YAW;
  */
   // m_gyro.configAllSettings(config); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_rightEncoder.getPosition();
    m_leftEncoder.getPosition();
    SmartDashboard.putNumber("left Encoders",getLeftDistance());
    SmartDashboard.putNumber("right Encoders", getRightDistance());
    SmartDashboard.putNumber("gyroangle", getAngle());
    SmartDashboard.putNumber("right Velocity", getRightVelocity());
    SmartDashboard.putNumber("left Velocity", getLeftVelocity());
    SmartDashboard.putNumber(" Velocity", getVelocity());
  }

  public void drive(Double forward, Double rotation){
    //maybe add gyro assist?
    m_drive.arcadeDrive(forward * NORMAL_MAX_FORWARD, rotation * NORMAL_MAX_TURN);
  }
  
  public void snailDrive(Double forward, Double rotation){
    m_drive.arcadeDrive(forward * SNAIL_MAX_FORWARD, rotation * SNAIL_MAX_TURN);

  }
  

  public void resetEncoders(){
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }
  
  public Rotation2d getRotation2D(){
   // return m_gyro.getRotation2d();
   return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }
  public Double getAngle(){
    return -m_gyro.getAngle();
  }
  
  public double getLeftDistance(){
    return m_leftEncoder.getPosition();
  }

  public double getRightDistance(){
    return -m_rightEncoder.getPosition();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getVelocity();
  }
  public double getRightVelocity(){
    return -m_rightEncoder.getVelocity();
  }

  public void tankDrive(Double rightVoltage, Double leftVoltage){
    m_rControllerGroup.setVoltage(rightVoltage);
    m_lControllerGroup.setVoltage(leftVoltage);
    m_drive.feed();
  }

  public void stopMotors(){
    m_rControllerGroup.setVoltage(0);
    m_lControllerGroup.setVoltage(0);
  }

  public double getRateAsRadians(){
   return m_gyro.getRate() * Math.PI/180;
  }

  public double getVelocity(){
    return (getLeftVelocity() + getRightVelocity())/2;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return new ChassisSpeeds(getVelocity(), 0, getRateAsRadians());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return KINEMATICS.toWheelSpeeds(this.getChassisSpeeds());
  }

  public void resetGyro(){
    m_gyro.reset();
  }
  public void calibrateGyro(){
    if(DriverStation.isDisabled()){
      m_gyro.calibrate();
    }
  }


  public void turnToTarget(double yaw, double setpoint){
    if(yaw >= 5){
    m_drive.arcadeDrive(0, TURN_TO_TARGET_CONTROLLER.calculate(yaw, setpoint));
    }
    else{
      m_drive.arcadeDrive(0, TURN_TO_TARGET_CONTROLLER.calculate(yaw, setpoint) + TURN_TO_TARGET_FF);
    }
  }

  public void chargeStationAlign(){
  /* 
     AUTO_BALANCE_CONTROLLER.calculate(KA * GRAVITY_VECTOR[2] * Math.sin(0),KA * GRAVITY_VECTOR[2] * Math.sin(m_gyro.getPitch()));
     //might have to negative this
     m_BLMotor.setVoltage(KA * GRAVITY_VECTOR[2] * Math.sin(m_gyro.getPitch()));
     m_BRMotor.setVoltage(KA * GRAVITY_VECTOR[2] * Math.sin(m_gyro.getPitch()));
     m_FRMotor.setVoltage(KA * GRAVITY_VECTOR[2] * Math.sin(m_gyro.getPitch()));
     m_FLMotor.setVoltage(KA * GRAVITY_VECTOR[2] * Math.sin(m_gyro.getPitch()));
     */
    /*
     double GRAVITY_VECTOR = m_gyro.getRawAccelZ();
     AUTO_BALANCE_CONTROLLER.calculate(KA * GRAVITY_VECTOR * Math.sin(0),KA * GRAVITY_VECTOR * Math.sin(m_gyro.getPitch()));
     */
  }

  public void setBrakeMode(){
    m_BLMotor.setIdleMode(IdleMode.kBrake);
    m_BRMotor.setIdleMode(IdleMode.kBrake);
    m_FRMotor.setIdleMode(IdleMode.kBrake);
    m_FLMotor.setIdleMode(IdleMode.kBrake);    
  }

  public void setCoastMode(){
    m_BLMotor.setIdleMode(IdleMode.kCoast);
    m_BRMotor.setIdleMode(IdleMode.kCoast);
    m_FRMotor.setIdleMode(IdleMode.kCoast);
    m_FLMotor.setIdleMode(IdleMode.kCoast);    
    
  }
  
  public static PPRamseteCommand followTrajCommand(DriveSubsystem driveSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      PathPlannerTrajectory trajectory ){
    return new PPRamseteCommand(
        trajectory, 
        poseEstimatorSubsystem::getPose2d, 
        RAMSETE_CONTROLLER, 
        KINEMATICS,
        driveSubsystem::tankDrive, 
        false);
  }
  
  


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
