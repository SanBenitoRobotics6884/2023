// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Drive.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ADIS16470_IMU;
import static frc.robot.util.ADIS16470_IMU.IMUAxis.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_FLMotor;
  private CANSparkMax m_FRMotor;
  private CANSparkMax m_BRMotor;
  private CANSparkMax m_BLMotor;
  private int m_state;
  private int m_debounceCount;

  private MotorControllerGroup m_rControllerGroup;
  private MotorControllerGroup m_lControllerGroup;
  private DifferentialDrive m_drive;

  
  Encoder m_rightEncoder;
  Encoder m_leftEncoder;
  //make sure to output degrees as negative!!!!!!!!!
 // WPI_Pigeon2 m_gyro;
 

ADIS16470_IMU m_gyro;

  

  ChassisSpeeds chassisSpeeds;
  public DriveSubsystem(ADIS16470_IMU gyro) {
    m_state = 0;
    m_debounceCount = 0;
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
    
    m_rightEncoder = new Encoder(RIGHT_CHANNEL_A, RIGHT_CHANNEL_B);
    m_leftEncoder = new Encoder(LEFT_CHANNEL_A, LEFT_CHANNEL_B);
    
    /*
    m_rightEncoder = new Encoder(BR_ID, BL_ID, false);
    m_leftEncoder = new Encoder(BR_ID, BL_ID, false);
    */

     m_rightEncoder.setReverseDirection(true);
     m_leftEncoder.setReverseDirection(false);


    m_rightEncoder.setDistancePerPulse(POSITION_CONVERSION);
    m_leftEncoder.setDistancePerPulse(POSITION_CONVERSION);
    

    m_leftEncoder.reset();
    m_rightEncoder.reset();
    
  
    m_gyro = gyro;
    
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
    m_rightEncoder.getDistance();
    m_leftEncoder.getDistance();
    SmartDashboard.putNumber("left Encoders",getLeftDistance());
    SmartDashboard.putNumber("right Encoders", getRightDistance());
    SmartDashboard.putNumber("gyroangle", getAngle());
    SmartDashboard.putNumber("right Velocity", getRightVelocity());
    SmartDashboard.putNumber("left Velocity", getLeftVelocity());
    SmartDashboard.putNumber(" Velocity", getVelocity());
    SmartDashboard.putNumber("pitch", this.getPitch());
    SmartDashboard.putNumber("chargeVoltage", KA*9.81*this.getPitch()/BALANCE_LIMITER);
    SmartDashboard.putNumber("Rate", this.getRateAsRadians());
  }

  public void drive(double forward, double rotation){
    //maybe add gyro assist?
    m_drive.arcadeDrive(forward * NORMAL_MAX_FORWARD, rotation * NORMAL_MAX_TURN);
  }
  
  public void snailDrive(double forward, double rotation){
    m_drive.arcadeDrive(forward * SNAIL_MAX_FORWARD, rotation * SNAIL_MAX_TURN);

  }
  

  public void resetEncoders(){
    m_rightEncoder.reset();
    m_leftEncoder.reset();
  }
  
  public Rotation2d getRotation2D(){
   // return m_gyro.getRotation2d();
   return Rotation2d.fromDegrees(this.getAngle());
  }
  //Should be CCW Positive
  public Double getAngle(){
    return -m_gyro.getAngle(kZ);
    
  }

  public Double getPitch(){
    return m_gyro.getAngle(kX);
  }
  
  public double getLeftDistance(){
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance(){
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity(){
    return m_leftEncoder.getRate();
  }
  public double getRightVelocity(){
    return m_rightEncoder.getRate();
  }

  public void tankDrive(Double rightVoltage, Double leftVoltage){
    m_rControllerGroup.setVoltage(-rightVoltage);
    m_lControllerGroup.setVoltage(-leftVoltage);
    m_drive.feed();
  }

  public void stopMotors(){
    m_rControllerGroup.setVoltage(0);
    m_lControllerGroup.setVoltage(0);
  }

  public double getRateAsRadians(){
   return -m_gyro.getRate(kZ) * Math.PI/180;
  }

  public double getVelocity(){
    return (getLeftVelocity() + getRightVelocity())/2;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return new ChassisSpeeds(getVelocity(), 0, this.getRateAsRadians());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return KINEMATICS.toWheelSpeeds(this.getChassisSpeeds());
  }

  public void resetGyro(){
    m_gyro.setGyroAngleZ(0);
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
    m_BLMotor.setVoltage(KA*9.81*this.getPitch()/BALANCE_LIMITER);
    m_BRMotor.setVoltage(-KA*9.81*this.getPitch()/BALANCE_LIMITER);
    m_FLMotor.setVoltage(KA*9.81*this.getPitch()/BALANCE_LIMITER);
    m_FRMotor.setVoltage(-KA*9.81*this.getPitch()/BALANCE_LIMITER);
    
  }
  public double autoBalanceRoutine() {
    switch (m_state) {
        // drive forwards to approach station, exit when tilt is detected
        case 0:
            if (this.getPitch() > ON_DEGREE) {
                m_debounceCount++;
            }
            if (m_debounceCount > this.secondsToTicks(BALANCE_DEBOUNCE_TIME)) {
                m_state = 1;
                m_debounceCount = 0;
                return BALANCE_SPEED_LOW;
            }
            return BALANCE_SPEED_HIGH;
        // driving up charge station, drive slower, stopping when level
        case 1:
            if (this.getPitch() < BALANCED_DEGREE) {
                m_debounceCount++;
            }
            if (m_debounceCount > secondsToTicks(BALANCE_DEBOUNCE_TIME)) {
                m_state = 2;
                m_debounceCount = 0;
                return 0;
            }
            return BALANCE_SPEED_LOW;
        // on charge station, stop motors and wait for end of auto
        case 2:
            if (Math.abs(this.getPitch()) <= BALANCED_DEGREE / 2) {
                m_debounceCount++;
            }
            if (m_debounceCount > secondsToTicks(BALANCE_DEBOUNCE_TIME)) {
                m_state = 4;
                m_debounceCount = 0;
                return 0;
            }
            if (this.getPitch() >= BALANCED_DEGREE) {
                return 0.1;
            } else if (this.getPitch() <= -BALANCED_DEGREE) {
                return -0.1;
            }
        case 3:
            return 0;
    }
    return 0;
}
 public void autoBalance(){
  double speed = this.autoBalanceRoutine();
  m_BLMotor.set(speed);
  m_BRMotor.set(-speed);
  m_FLMotor.set(speed);
  m_FRMotor.set(-speed);
 }
public int secondsToTicks(double time) {
  return (int) (time * 50);
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
  
  public  SequentialCommandGroup followAutoCommand(DriveSubsystem m_driveSubsystem,
      PoseEstimatorSubsystem poseEstimatorSubsystem,
      List<PathPlannerTrajectory> trajectory, HashMap<String, Command>m_hashMap ){
        poseEstimatorSubsystem.ResetPose2d(trajectory.get(0).getInitialPose());
      
      
        RamseteAutoBuilder autoBuilder =
         new RamseteAutoBuilder(poseEstimatorSubsystem::getPose2d, poseEstimatorSubsystem::ResetPose2d, RAMSETE_CONTROLLER,
          KINEMATICS, FEED_FOWARD, m_driveSubsystem::getWheelSpeeds, new PIDConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD), m_driveSubsystem::tankDrive,
           m_hashMap, m_driveSubsystem);
       

        Command auto = autoBuilder.followPathGroupWithEvents(trajectory);

        return new SequentialCommandGroup(auto
        ,
          new RunCommand(m_driveSubsystem::stopMotors, m_driveSubsystem));
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
