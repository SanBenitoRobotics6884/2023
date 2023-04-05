// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.RobotConstants.Intake.*;

/**
 * Button 3---Launch cube
 * Button 4---Pick up cube
 * Button 2---Stop (not super necessary, stops on its own)
 */
public class IntakeSubsystem extends SubsystemBase {
  private enum IntakeStatus {
    INHALING,
    EXHALING,
    STOPPED;
  }
  IntakeStatus m_status = IntakeStatus.STOPPED;
  CANSparkMax m_motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  RelativeEncoder m_encoder = m_motor.getEncoder();
  double m_timestamp = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    
    UsbCamera m_intakeCamera = CameraServer.startAutomaticCapture(0);
    m_intakeCamera.setResolution(320, 240);
    m_intakeCamera.setFPS(15);

    UsbCamera m_topCamera = CameraServer.startAutomaticCapture(1);
    m_topCamera.setResolution(320, 240);
    m_topCamera.setFPS(15);
  }

  @Override
  public void periodic() {
    double output;
    switch (m_status) {
      case INHALING:
        output = INHALE_VOLTAGE;
        break;
      case EXHALING:
        output = EXHALE_VOLTAGE;
        break;
      default:
        output = 0;
    }
    m_motor.set(output);

    SmartDashboard.putNumber("Speed", m_encoder.getVelocity());
    SmartDashboard.putNumber("output", output);
  }

  public void startIntaking() {
    m_status = IntakeStatus.INHALING;
  }

  public void startExhaling() {
    m_status = IntakeStatus.EXHALING;
  }

  public void stop() {
    m_status = IntakeStatus.STOPPED;
  }

  public Command getInhaleCommand() {
    return runOnce(this::startIntaking);
  }

  public Command getExhaleCommand() {
    return runOnce(this::startExhaling);
  }

  public Command getStopCommand() {
    return runOnce(this::stop);
  }
}
