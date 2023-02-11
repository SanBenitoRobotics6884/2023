// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ClawSubsystem;
import static frc.robot.Constants.Claw.*;

import java.util.function.BooleanSupplier;

public class ClawCommand extends CommandBase {
  ClawSubsystem m_clawSubsystem;
  boolean m_triggerState = false;
  boolean m_leftPressed = false;
  boolean m_rightPressed = false;
  double m_rotations = 0;
  double m_CurrentButton = 0;

  public ClawCommand(
      ClawSubsystem clawSubsystem, 
      BooleanSupplier triggerPressed, 
      BooleanSupplier leftPressed, 
      BooleanSupplier rightPressed) 
  {
    addRequirements(clawSubsystem);
    m_clawSubsystem = clawSubsystem;
    m_triggerState = triggerPressed.getAsBoolean();
    m_leftPressed = triggerPressed.getAsBoolean();
    m_rightPressed = triggerPressed.getAsBoolean();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_triggerState) {
     if (m_leftPressed && m_rotations < kContLimit - 0.01) {
        m_rotations += kCloseClawRate;
      } else if (m_rightPressed && m_rotations < kContLimit + 0.01) {
        m_rotations += kOpenClawRate;
      }
   } else {
    if (m_leftPressed) {
      m_rotations = kCubeClose;
    } else if (m_rightPressed) {
      m_rotations = kConeClose;
    }
   }
    m_clawSubsystem.setRotations(m_rotations);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
