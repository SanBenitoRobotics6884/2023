
package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ADIS16470_IMU;

import static frc.robot.util.ADIS16470_IMU.IMUAxis.*;
import static frc.robot.constants.RobotConstants.Drive.*;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_FLMotor;
  private CANSparkMax m_FRMotor;
  private CANSparkMax m_BRMotor;
  private CANSparkMax m_BLMotor;
  private MotorControllerGroup m_rControllerGroup;
  private MotorControllerGroup m_lControllerGroup;
  private DifferentialDrive m_drive;
  private Encoder m_rightEncoder;
  private Encoder m_leftEncoder;
  private ADIS16470_IMU m_gyro;
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

    m_BLMotor.setIdleMode(IdleMode.kBrake);
    m_BRMotor.setIdleMode(IdleMode.kBrake);
    m_FRMotor.setIdleMode(IdleMode.kBrake);
    m_FLMotor.setIdleMode(IdleMode.kBrake);  

    m_rControllerGroup.setInverted(true);
    m_lControllerGroup.setInverted(false);
   

    m_drive = new DifferentialDrive(m_lControllerGroup, m_rControllerGroup);
    
    m_rightEncoder = new Encoder(RIGHT_CHANNEL_A, RIGHT_CHANNEL_B);
    m_leftEncoder = new Encoder(LEFT_CHANNEL_A, LEFT_CHANNEL_B);

     m_rightEncoder.setReverseDirection(false);
     m_leftEncoder.setReverseDirection(false);


    m_rightEncoder.setDistancePerPulse(POSITION_CONVERSION);
    m_leftEncoder.setDistancePerPulse(POSITION_CONVERSION);
    

    m_leftEncoder.reset();
    m_rightEncoder.reset();
    
  
    m_gyro = gyro;
    
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
    m_drive.arcadeDrive(-forward * NORMAL_MAX_FORWARD, rotation * NORMAL_MAX_TURN);
  }
  
  public void snailDrive(double forward, double rotation){
    m_drive.arcadeDrive(-forward * SNAIL_MAX_FORWARD, rotation * SNAIL_MAX_TURN);

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

  public void tankDrive(Double leftVoltage, Double rightVoltage){
    m_rControllerGroup.setVoltage(rightVoltage);
    m_lControllerGroup.setVoltage(leftVoltage);
    m_drive.feed();
  }
  public void testDrive(){
    m_rControllerGroup.setVoltage(1);
    m_lControllerGroup.setVoltage(1);
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
    if(DriverStation.isDisabled())
      m_gyro.calibrate();
    
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
   double m_balanceOut = MathUtil.clamp(AUTO_BALANCE_CONTROLLER.calculate(this.getPitch(), 0), -BALANCE_MAX_OUTPUT, BALANCE_MAX_OUTPUT);
   m_drive.arcadeDrive(m_balanceOut, 0);
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


  public static PPRamseteCommand followTrajCommand(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem,
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
