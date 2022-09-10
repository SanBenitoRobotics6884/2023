// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  Translation2d m_frontLeftLocation = new Translation2d(0.254, 0.254);
  Translation2d m_frontRightLocation = new Translation2d(0.254, -0.254);
  Translation2d m_backLeftLocation = new Translation2d(-0.254, 0.254);
  Translation2d m_backRightLocation = new Translation2d(-0.254, -0.254);

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d());
  Pose2d m_pose = new Pose2d();

  private static final double kSlowspd = 0.4;
  private static final double kFastSpd = 0.8;
  private static final double kLwrStorageDelay = 3;
  private static final double kDriveGearing = 10.71;
  private static final double kWheelCircummference = 6 * Math.PI;
  private static final double intkVltge = 6;
  private static final double uprMtrSpd = 0.75;
  private static final double lwrMtrSpd = 0.6;
  private static final double kdrivedeadband = 0.1;
  private static final double kMaxTurn = 360;
  private static final double kP = 0.008;
  private static final double kD = 0.00005;
  private static final double kF = 0.1;

  private static final double kPlift = 0.00015;
  private static final double kIlift = 0;
  private static final double kDlift = 0;
  private static final double kFUnloadedLift = 0;
  private static final double kFLoadedLift = 0.0004;
  private static final double kMinOutputLift = -1;
  private static final double kMaxOutputLift = 1;
  private static final double maxVelLift = 2000; // rpm
  private static final double maxAccLift = 1500;
  private static final double allowedErrLift = 0;

  private static final boolean isPid = false; //Run in PID mode instead of direct control (open loop)
  private static final boolean isDynamicKF = false; //EXPERIMENTAL. USE WITH CAUTION
  private static final double kLowSetpoint = 0; //Units: motor rotations
  private static final double kHighSetpoint = 20; //Units: motor rotations
  private static final double kMaxVoltageLeft = -4; //CHANGE BACK TO 4
  private static final double kMaxVoltageRight = -4.5; //CHANGE BACK TO 4
  private static final double kRatchetDeploy = 1;
  private static final double kRatchetRetract = -0.35;
  private static final double kRatchetDelay = 3;

  private static final int smartMotionSlot = 0;
  private static final int kLeftLiftID = 8;
  private static final int kRightLiftID = 9;
  private static final int kJoystickPort = 0;
  private static final int kLeftServoPort = 0;
  private static final int kRightServoPort = 1;

  //Rumble constants
  private static final double kRumblePulseWidth = 0.3; // Duration of rumble pulse
  private static final double kRumblePulseRate = 3;
  private static final double kRumbleStrength = 1;
  private static final double kAccelerationRumbleThreshold = 8.5;
  private static final double kCurrentRatioRumbleThreshold = 3;
  private static final int kDrivePowerChannels[] = {4,7,16,19};

  private boolean isEvac = false;
  private boolean isLaunching = false;
  private boolean isIntaking = false;
  private boolean prevTrigger = false;
  private double lwrStorageTargetTime = 0;
  private double turnPID = 0;
  private double maxDriveSpdScalar = 0.75;
  private double netAccelertion = 0;

  private Joystick m_joystick = new Joystick(kJoystickPort);
  private XboxController m_controller = new XboxController(1);

  private WPI_VictorSPX m_uprStorageMtr = new WPI_VictorSPX(7);
  private WPI_VictorSPX m_lwrStorageMtr = new WPI_VictorSPX(5);

  private CANSparkMax m_leftLiftMtr = new CANSparkMax(kLeftLiftID, MotorType.kBrushless);
  private CANSparkMax m_rightLiftMtr = new CANSparkMax(kRightLiftID, MotorType.kBrushless);
  private SparkMaxPIDController m_leftLiftPid;
  private SparkMaxPIDController m_rightLiftPid;
  private RelativeEncoder m_leftLiftEncoder;
  private RelativeEncoder m_rightLiftEncoder;

  private Servo m_leftAcuator = new Servo(kLeftServoPort);
  private Servo m_rightAcuator = new Servo(kRightServoPort);

  DigitalInput m_leftLimit = new DigitalInput(8);
  DigitalInput m_rightLimit = new DigitalInput(9);

  private double targetLiftTime = 0;
  private double prevLiftSpeed = 0;
  private boolean prevRatchet = true;
  private int closedLoopMode = 0; // 1: High, 0: Low

  private CANSparkMax m_leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_rightFront = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax m_leftBack = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rightBack = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax m_intakeMotor = new CANSparkMax(6, MotorType.kBrushless);

  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private RelativeEncoder m_backLeftEncoder;
  private RelativeEncoder m_backRightEncoder;

  private MecanumDrive m_drive = new MecanumDrive(m_leftFront, m_leftBack, m_rightFront, m_rightBack);
  private ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private PIDController m_TurnPID = new PIDController(kP, 0, kD);
  private PowerDistribution m_pdh = new PowerDistribution();
  

  enum DriveMode{
    DEFAULT,
    FIELD_CENTRIC,
    GYRO_ASSIST,
    GYRO_ASSIST_FIELD_CENTER
  }

  private static final DriveMode mode = DriveMode.DEFAULT;
 
  @Override
  public void robotInit() {
    m_leftLiftMtr.restoreFactoryDefaults();
    m_rightLiftMtr.restoreFactoryDefaults();

    m_leftLiftMtr.setInverted(true);
    m_rightLiftMtr.setInverted(false);

    m_leftAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    m_rightAcuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    m_rightLiftEncoder = m_rightLiftMtr.getEncoder();
    m_rightLiftPid = m_rightLiftMtr.getPIDController();
    m_rightLiftPid.setP(kPlift);
    m_rightLiftPid.setI(kIlift);
    m_rightLiftPid.setD(kDlift);
    m_rightLiftPid.setIZone(0);
    m_rightLiftPid.setFF(kFUnloadedLift);
    m_rightLiftPid.setOutputRange(kMinOutputLift, kMaxOutputLift);

    m_rightLiftPid.setSmartMotionMaxVelocity(maxVelLift, smartMotionSlot);
    m_rightLiftPid.setSmartMotionMaxAccel(maxAccLift, smartMotionSlot);
    m_rightLiftPid.setSmartMotionAllowedClosedLoopError(allowedErrLift, smartMotionSlot);

    m_leftLiftEncoder = m_leftLiftMtr.getEncoder();
    m_leftLiftPid = m_leftLiftMtr.getPIDController();
    m_leftLiftPid.setP(kPlift);
    m_leftLiftPid.setI(kIlift);
    m_leftLiftPid.setD(kDlift);
    m_leftLiftPid.setIZone(0);
    m_leftLiftPid.setFF(kFUnloadedLift);
    m_leftLiftPid.setOutputRange(kMinOutputLift, kMaxOutputLift);

    m_leftLiftPid.setSmartMotionMaxVelocity(maxVelLift, smartMotionSlot);
    m_leftLiftPid.setSmartMotionMaxAccel(maxAccLift, smartMotionSlot);
    m_leftLiftPid.setSmartMotionAllowedClosedLoopError(allowedErrLift, smartMotionSlot);

    //Deploy ratchets at match start
    m_leftAcuator.setSpeed(kRatchetDeploy);
    m_rightAcuator.setSpeed(kRatchetDeploy);

    m_lwrStorageMtr.configFactoryDefault();
    m_uprStorageMtr.configFactoryDefault();
    m_leftBack.restoreFactoryDefaults();
    m_leftFront.restoreFactoryDefaults();
    m_rightBack.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    

    m_frontLeftEncoder = m_leftFront.getEncoder();
    m_frontRightEncoder = m_rightFront.getEncoder();
    m_backLeftEncoder = m_leftBack.getEncoder();
    m_backRightEncoder = m_rightBack.getEncoder();

    m_frontLeftEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_frontRightEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_backLeftEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);
    m_backRightEncoder.setVelocityConversionFactor(kWheelCircummference / kDriveGearing);

    m_frontLeftEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_frontRightEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_backLeftEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);
    m_backRightEncoder.setPositionConversionFactor(kWheelCircummference / kDriveGearing);

    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);
    m_lwrStorageMtr.setInverted(true);
    m_uprStorageMtr.setNeutralMode(NeutralMode.Coast);

    m_gyro.calibrate();
    m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);
    m_TurnPID.disableContinuousInput();

    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    CameraServer.startAutomaticCapture(2);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Position", m_leftLiftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightLiftEncoder.getPosition());
    SmartDashboard.putBoolean("Right LS", m_rightLimit.get());
    SmartDashboard.putBoolean("Left LS", m_leftLimit.get());

    if (!m_leftLimit.get()) {
      m_leftLiftEncoder.setPosition(0);
    }
    if (!m_rightLimit.get()) {
      m_rightLiftEncoder.setPosition(0);
    }

    if (m_controller.getLeftBumper()) {
      maxDriveSpdScalar = kSlowspd;
   }
   if (m_controller.getRightBumper()) {
      maxDriveSpdScalar = kFastSpd;
   }

    if (m_joystick.getTrigger()) {
      isLaunching = true;
    } else {
      isLaunching = false;
    }
    if (m_joystick.getRawButton(2)) {
      isIntaking = true;
    } else {
      isIntaking = false;
    }
    if (m_joystick.getRawButton(7)) {
      isEvac = true;
    } else {
      isEvac = false;
    }
    
    // Calculate net acceleration (all directions) and subtract acc due to gravity
    netAccelertion = Math.sqrt(Math.pow(m_gyro.getAccelX(), 2) +
      Math.pow(m_gyro.getAccelY(), 2) + Math.pow(m_gyro.getAccelZ(), 2))-9.8;
    netAccelertion = Math.abs(netAccelertion);

    // Deadband net acceleration for random fluctuations
    if (netAccelertion < 0.05){
      netAccelertion = 0; 
    }

    // Calculate avg current from drive motors only
    double avgDriveCurrent = ( m_pdh.getCurrent(kDrivePowerChannels[0]) +
                          m_pdh.getCurrent(kDrivePowerChannels[1]) +
                          m_pdh.getCurrent(kDrivePowerChannels[2]) +
                          m_pdh.getCurrent(kDrivePowerChannels[3])
                        ) / 4;
    double currentRatio = (avgDriveCurrent / netAccelertion);

    // Pulse if robot is pushing against obstacle or other robot
    boolean isPulsing;
    if (currentRatio > kCurrentRatioRumbleThreshold) {
      isPulsing = true;
    } else {
      isPulsing = false;
    }

    //If pulsing, alternate between full rumble and no rumble
    if (isPulsing) {
      double pulseTime = (Timer.getFPGATimestamp() * kRumblePulseRate) % 1;

      if (pulseTime < kRumblePulseWidth) {
        m_controller.setRumble(RumbleType.kLeftRumble, kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, -kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, -kRumbleStrength);
      }
    //Otherwise rumble when past the acceleration threshold
    } else {
      if (netAccelertion > kAccelerationRumbleThreshold) {
        m_controller.setRumble(RumbleType.kLeftRumble, kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
      }
    }

    SmartDashboard.putNumber("Acceleration", netAccelertion);

    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putData("drive", m_drive);
    SmartDashboard.putNumber("pose X", m_pose.getX());
    SmartDashboard.putNumber("pose Y", m_pose.getY());
    SmartDashboard.putNumber("PID Out", turnPID);

    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      Units.inchesToMeters(m_frontLeftEncoder.getVelocity()), Units.inchesToMeters(m_frontRightEncoder.getVelocity()),
      Units.inchesToMeters(m_backLeftEncoder.getVelocity()), Units.inchesToMeters(m_backRightEncoder.getVelocity()));

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

    // Update the pose
    m_pose = m_odometry.update(gyroAngle, wheelSpeeds);
  }

  @Override
  public void autonomousInit() {
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (isPid) {
      if (m_joystick.getTrigger()) {
        closedLoopLift();
      } else {
        m_leftLiftMtr.setVoltage(0);
        m_rightLiftMtr.setVoltage(0);
      }
    } else {
      openLoopLift(m_joystick.getY());
    }


    double yspeed = -m_controller.getLeftY() * Math.abs(m_controller.getLeftY());
    double xspeed = m_controller.getLeftX() * Math.abs(m_controller.getLeftX());
    double zrot = m_controller.getRightX() * Math.abs(m_controller.getRightX());

    if (Math.abs(yspeed) < kdrivedeadband) yspeed = 0;
    if (Math.abs(xspeed) < kdrivedeadband) xspeed = 0;
    if (Math.abs(zrot) < kdrivedeadband) zrot = 0;

    yspeed *= maxDriveSpdScalar;
    xspeed *= maxDriveSpdScalar;
    zrot *= maxDriveSpdScalar;

    turnPID = m_TurnPID.calculate(m_gyro.getRate(), zrot * kMaxTurn);
    if (turnPID > 0) {
      turnPID -= kF;
    } else if (turnPID < 0) {
      turnPID += kF;
    }

    if (mode == DriveMode.DEFAULT) {
      m_drive.driveCartesian(yspeed, xspeed, zrot);
    } else if (mode == DriveMode.FIELD_CENTRIC){
      m_drive.driveCartesian(yspeed, xspeed, zrot, m_gyro.getAngle());
    } else if (mode == DriveMode.GYRO_ASSIST) {

    } else if (mode == DriveMode.GYRO_ASSIST_FIELD_CENTER) {

    }

    if (isIntaking) {
      m_lwrStorageMtr.set(ControlMode.PercentOutput, lwrMtrSpd);
      m_intakeMotor.setVoltage(intkVltge);
    } else {
      m_lwrStorageMtr.set(ControlMode.PercentOutput, 0);
      m_intakeMotor.setVoltage(0);
    }

    if (isLaunching ) {
      m_uprStorageMtr.set(ControlMode.PercentOutput, uprMtrSpd);
      if (! prevTrigger){
        lwrStorageTargetTime = Timer.getFPGATimestamp()+kLwrStorageDelay;
      }

      if (Timer.getFPGATimestamp() > lwrStorageTargetTime){
        m_lwrStorageMtr.set(ControlMode.PercentOutput, lwrMtrSpd);
      } else m_lwrStorageMtr.set(ControlMode.PercentOutput, 0);
    } else {
      m_uprStorageMtr.set(ControlMode.PercentOutput, 0);
    }

    if(isEvac) {
      m_intakeMotor.setVoltage(-intkVltge);
      m_lwrStorageMtr.set(-lwrMtrSpd);
      m_uprStorageMtr.set(-lwrMtrSpd);
    }

    prevTrigger = m_joystick.getTrigger();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (m_joystick.getRawButton(3)) {
      m_leftAcuator.setSpeed(m_joystick.getY());
      SmartDashboard.putNumber("Left Retract", m_joystick.getY());
    } else if (m_joystick.getRawButton(4)) {
      m_rightAcuator.setSpeed(m_joystick.getY());
      SmartDashboard.putNumber("Right Retract", m_joystick.getY());
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

public void openLoopLift(double speed) {
  //Deadband
  if (Math.abs(speed) < 0.1) {
    speed = 0;
  }

  SmartDashboard.putNumber("Speed", speed * kMaxVoltageLeft);

  if (speed <= 0) { //Retracting Lift
    m_leftAcuator.setSpeed(kRatchetDeploy);
    m_rightAcuator.setSpeed(kRatchetDeploy);

    if (!m_joystick.getRawButton(4) && m_leftLimit.get()) {
      m_leftLiftMtr.setVoltage(speed * kMaxVoltageLeft);
    } else {
      m_leftLiftMtr.setVoltage(0);
    }
    if (!m_joystick.getRawButton(3) && m_rightLimit.get()) {
      m_rightLiftMtr.setVoltage(speed * kMaxVoltageRight);
    } else {
      m_rightLiftMtr.setVoltage(0);
    }

  } else { //Raising Lift
    //Ratchets must be fully retracted to raise lift arms
    m_leftAcuator.setSpeed(kRatchetRetract);
    m_rightAcuator.setSpeed(kRatchetRetract);

    //Allow arms to move after 1 second has passed
    if (prevLiftSpeed <= 0) {
      targetLiftTime = Timer.getFPGATimestamp() + kRatchetDelay;
    }
    if (Timer.getFPGATimestamp() >= targetLiftTime) {
      if (!m_joystick.getRawButton(4)) {
        m_leftLiftMtr.setVoltage(speed * kMaxVoltageLeft);
      } else {
        m_leftLiftMtr.setVoltage(0);
      }
      if (!m_joystick.getRawButton(3)) {
        m_rightLiftMtr.setVoltage(speed * kMaxVoltageRight);
      } else {
        m_rightLiftMtr.setVoltage(0);
      }
    } else {
      m_leftLiftMtr.setVoltage(0);
      m_rightLiftMtr.setVoltage(0);
    }

  }

  prevLiftSpeed = speed;

}

public void closedLoopLift() {

  if (m_joystick.getRawButton(4)) {
    closedLoopMode = 0;
  }

  if (m_joystick.getRawButton(6)) {
    closedLoopMode = 1;
  }

  if (closedLoopMode == 0) { //Retracting
    //If dynamic kF enabled, use higher FF gain to counteract
    //weight of robot while lifting
    if (prevRatchet == false && isDynamicKF) {
      m_leftLiftPid.setFF(kFLoadedLift);
      m_rightLiftPid.setFF(kFLoadedLift);
    }

    //Set target position to retract and deploy ratchet
    m_leftLiftPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
    m_rightLiftPid.setReference(kLowSetpoint, CANSparkMax.ControlType.kSmartMotion);
    m_leftAcuator.setSpeed(kRatchetDeploy);
    m_rightAcuator.setSpeed(kRatchetDeploy);
    prevRatchet = true;
  }


  if (closedLoopMode == 1) { //Extending
    //Use default FF gain when lifting (no weight to account for)
    if (prevRatchet == true && isDynamicKF) {
      m_leftLiftPid.setFF(kFUnloadedLift);
      m_rightLiftPid.setFF(kFUnloadedLift);
    }

    m_leftAcuator.setSpeed(kRatchetRetract);
    m_rightAcuator.setSpeed(kRatchetRetract);

    //Delay 1 second so ratchets can fully deploy
    if (prevRatchet == true) {
      targetLiftTime = Timer.getFPGATimestamp() + kRatchetDelay;
    }
    if (Timer.getFPGATimestamp() >= targetLiftTime) {
      m_leftLiftPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
      m_rightLiftPid.setReference(kHighSetpoint, CANSparkMax.ControlType.kSmartMotion);
      System.out.println("Run");
    } else {
      m_leftLiftMtr.setVoltage(0);
      m_rightLiftMtr.setVoltage(0);
    }
    prevRatchet = false;
  }
}
}
