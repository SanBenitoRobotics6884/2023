// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Feedback.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedbackSubsystem extends SubsystemBase {
  PivotSubsystem m_pivotSubsystem;
  ExtendSubsystem m_extendSubsystem;
  ClawSubsystem m_clawSubsystem;
  DriveSubsystem m_driveSubsystem;
  VisionSubsystem m_visionSubsystem;
  ADIS16470_IMU m_gyro;
  XboxController m_controller;
  PowerDistribution m_pdh;
  UsbCamera m_clawCam;

  double m_currentToAccelerationRatio = 0;
  double m_netAcceleration = 0;

  /** Creates a new FeedbackSubsystem. */
  public FeedbackSubsystem(
      PivotSubsystem pivotSubsystem,
      ExtendSubsystem extendSubsystem,
      ClawSubsystem clawSubsystem,
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      ADIS16470_IMU gyro,
      XboxController controller,
      PowerDistribution pdh) {
    m_pivotSubsystem = pivotSubsystem;
    m_extendSubsystem = extendSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_gyro = gyro;
    m_controller = controller;
    m_pdh = pdh;

    m_clawCam = CameraServer.startAutomaticCapture();
    m_clawCam.setResolution(320, 240);
    m_clawCam.setFPS(20);
  }

  @Override
  public void periodic() {
    // Calculate net acceleration
    m_netAcceleration = Math.sqrt(
      Math.pow(m_gyro.getAccelX(), 2) +
      Math.pow(m_gyro.getAccelY(), 2) +
      Math.pow(m_gyro.getAccelZ() - GRAVITY, 2)
    );

    // Calculates the ratio of the average current in drive motors to the robot's acceleration
    m_currentToAccelerationRatio = 0.25 * (
        m_pdh.getCurrent(DRIVE_POWER_CHANNEL[0])
        + m_pdh.getCurrent(DRIVE_POWER_CHANNEL[1])
        + m_pdh.getCurrent(DRIVE_POWER_CHANNEL[2])
        + m_pdh.getCurrent(DRIVE_POWER_CHANNEL[3])
    ) / m_netAcceleration;

    // Set rumble of controller. Values of STRENGTH may vary depending on turret or claw.
    if (m_clawSubsystem.getStatus() 
        || isHittingObstacle()
        || isOverAccelerationThreshold()) {
      m_controller.setRumble(RumbleType.kLeftRumble, STRENGTH);
      m_controller.setRumble(RumbleType.kRightRumble, STRENGTH);
    } else {
      m_controller.setRumble(RumbleType.kLeftRumble, 0);
      m_controller.setRumble(RumbleType.kRightRumble, 0);
    }
    
      SmartDashboard.putNumber("Arm Angle Measurement", m_pivotSubsystem.getPosition());
      SmartDashboard.putNumber("Arm Extension Measurement", m_extendSubsystem.getPosition());
      SmartDashboard.putNumber("Net Acceleration", m_netAcceleration);
      SmartDashboard.putNumber("Gyro Yaw", m_gyro.getAngle());
      SmartDashboard.putNumber("Gyro accZ", m_gyro.getAccelZ());
      SmartDashboard.putNumber("Target Yaw", m_visionSubsystem.getYawDegrees());
      SmartDashboard.putNumber("Target Pitch", m_visionSubsystem.getPitchDegrees());
      SmartDashboard.putNumber("Current To Acceleration Ratio", m_currentToAccelerationRatio);
      SmartDashboard.putNumber("Claw Camera FPS", m_clawCam.getActualFPS());
      SmartDashboard.putBoolean("Claw Camera Connected", m_clawCam.isConnected());
      // Get motor temperatures 

      SmartDashboard.putString("Claw Motor Temperature", m_clawSubsystem.getMotorTemp() + "°F, " + 
      m_clawSubsystem.isDangerTemp());
      SmartDashboard.putString("Pivot Master Motor Temperature", m_pivotSubsystem.getMMotorTemperature() + "°F, " + 
      m_pivotSubsystem.isDangerTemp(m_pivotSubsystem.getMMotorTemperature()));
      SmartDashboard.putString("Pivot Slave Motor Temperature", m_pivotSubsystem.getSMotorTemperature() + "°F, " + 
      m_pivotSubsystem.isDangerTemp(m_pivotSubsystem.getSMotorTemperature()));
      SmartDashboard.putString("Extension Motor Temperature", m_extendSubsystem.getMotorTemperature() + "°F, " + 
      m_extendSubsystem.isDangerTemp());
      SmartDashboard.putString("Front Left Motor Temperature", m_driveSubsystem.getFLMotorTemperature() + "°F, " + 
      m_driveSubsystem.isDangerTemp(m_driveSubsystem.getFLMotorTemperature()));
      SmartDashboard.putString("Front Left Motor Temperature", m_driveSubsystem.getFRMotorTemperature() + "°F, " + 
      m_driveSubsystem.isDangerTemp(m_driveSubsystem.getFRMotorTemperature()));
      SmartDashboard.putString("Front Left Motor Temperature", m_driveSubsystem.getBLMotorTemperature() + "°F, " + 
      m_driveSubsystem.isDangerTemp(m_driveSubsystem.getBLMotorTemperature()));
      SmartDashboard.putString("Front Left Motor Temperature", m_driveSubsystem.getBRMotorTemperature() + "°F, " + 
      m_driveSubsystem.isDangerTemp(m_driveSubsystem.getBRMotorTemperature()));

      // Get gyro values (already done in driveSubsystem, move later)
  }

  private boolean isHittingObstacle() {
    return m_currentToAccelerationRatio > CURRENT_RATIO_RUMBLE_THRESHOLD;
  }

  private boolean isOverAccelerationThreshold() {
    return m_netAcceleration > ACCELERATION_RUMBLE_THRESHOLD;
  }

  public double celciusToFahrenheit(double celcius) {
    double fahrenheit = (celcius * 9.0 / 5.0) + 32;
    return fahrenheit;
  }
}