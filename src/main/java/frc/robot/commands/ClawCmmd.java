// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import static frc.robot.ConstantsFolder.RobotConstants.Claw.*;

import java.util.function.BooleanSupplier;

public class ClawCmmd extends CommandBase {
  ClawSubsystem m_clawSubsystem;
  BooleanSupplier m_triggerState;
  BooleanSupplier m_leftPressed;
  BooleanSupplier m_rightPressed;
  double m_rotations = 0;
  double m_CurrentButton = 0;

  public ClawCmmd(
      ClawSubsystem clawSubsystem, 
      BooleanSupplier triggerPressed, 
      BooleanSupplier leftPressed, 
      BooleanSupplier rightPressed) 
  {
    addRequirements(clawSubsystem);
    m_clawSubsystem = clawSubsystem;
    m_triggerState = triggerPressed;
    m_leftPressed = leftPressed;
    m_rightPressed = rightPressed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_triggerState.getAsBoolean()) {
     if (m_leftPressed.getAsBoolean() && m_rotations < kContLimit - 0.01) {
        m_rotations += kCloseClawRate;
      } else if (m_rightPressed.getAsBoolean() && m_rotations < kContLimit + 0.01) {
        m_rotations += kOpenClawRate;
      }
   } else {
    if (m_leftPressed.getAsBoolean()) {
      m_rotations = kCubeClose;
    } else if (m_rightPressed.getAsBoolean()) {
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
