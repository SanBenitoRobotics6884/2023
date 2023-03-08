// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

import static frc.robot.constants.RobotConstants.Claw.*;

import java.util.function.BooleanSupplier;

public class ClawCmmd extends CommandBase {
  ClawSubsystem m_clawSubsystem;
  BooleanSupplier m_triggerState;
  BooleanSupplier m_leftPressed;
  BooleanSupplier m_rightPressed;

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
    double rotations = m_clawSubsystem.getRotations();
    double newSetpoint;
    boolean triggerPressed = m_triggerState.getAsBoolean();
    boolean leftPressed = m_leftPressed.getAsBoolean();
    boolean rightPressed = m_rightPressed.getAsBoolean();
    if (!triggerPressed) {
      // When the trigger and left button are pressed together, close claw
      newSetpoint = rotations + OPEN_RATE;
      if (leftPressed && !rightPressed 
          && OPEN_SETPOINT >= newSetpoint && newSetpoint >= CLOSED_SETPOINT) {
        rotations = newSetpoint;
      }
      // When the trigger and right button are pressed together, open claw
      newSetpoint = rotations + CLOSE_RATE;
      if (rightPressed && !leftPressed 
          && OPEN_SETPOINT >= newSetpoint && newSetpoint >= CLOSED_SETPOINT) {
        rotations = newSetpoint;
      }
    } else {
      if (leftPressed && !rightPressed) {
        rotations = CUBE_SETPOINT;
      } else if (rightPressed && !leftPressed) {
        rotations = CONE_SETPOINT;
      }
    }
    m_clawSubsystem.setRotations(rotations);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
