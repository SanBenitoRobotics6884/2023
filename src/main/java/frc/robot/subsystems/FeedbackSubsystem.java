// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Feedback.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedbackSubsystem extends SubsystemBase {
  ArmSubsystem m_armSubsystem;
  ClawSubsystem m_clawSubsystem;
  VisionSubsystem m_visionSubsystem;
  ADIS16470_IMU m_gyro;
  XboxController m_controller;
  PowerDistribution m_pdh;
  UsbCamera m_clawCam;

  double m_currentToAccelerationRatio = 0;
  double m_netAcceleration = 0;

  /** Creates a new FeedbackSubsystem. */
  public FeedbackSubsystem(
      ArmSubsystem armSubsystem,
      ClawSubsystem clawSubsystem,
      VisionSubsystem visionSubsystem,
      ADIS16470_IMU gyro,
      XboxController controller,
      PowerDistribution pdh) {
    m_armSubsystem = armSubsystem;
    m_clawSubsystem = clawSubsystem;
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
    
    if (IS_TESTING) {
      SmartDashboard.putNumber("Arm Angle Measurement", m_armSubsystem.getPivotPosition());
      SmartDashboard.putNumber("Arm Extension Measurement", m_armSubsystem.getExtendPosition());
      SmartDashboard.putNumber("Net Acceleration", m_netAcceleration);
      SmartDashboard.putNumber("Gyro Yaw", m_gyro.getAngle());
      SmartDashboard.putNumber("Gyro accZ", m_gyro.getAccelZ());
      SmartDashboard.putNumber("Target Yaw", m_visionSubsystem.getYawDegrees());
      SmartDashboard.putNumber("Target Pitch", m_visionSubsystem.getPitchDegrees());
      SmartDashboard.putNumber("Current To Acceleration Ratio", m_currentToAccelerationRatio);
      SmartDashboard.putNumber("Claw Camera FPS", m_clawCam.getActualFPS());
      SmartDashboard.putBoolean("Claw Camera Connected", m_clawCam.isConnected());
    }  
  }

  private boolean isHittingObstacle() {
    return m_currentToAccelerationRatio > CURRENT_RATIO_RUMBLE_THRESHOLD;
  }

  private boolean isOverAccelerationThreshold() {
    return m_netAcceleration > ACCELERATION_RUMBLE_THRESHOLD;
  }
}
