// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

import static frc.robot.constants.RobotConstants.LED.*;

public class BlinkLEDCommand extends CommandBase {
  LEDSubsystem m_ledSubsystem;
  Timer m_timer;

  /** Creates a new BlinkLEDCommand. */
  public BlinkLEDCommand(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_ledSubsystem.setConformistMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(TELEOP_BLINK_TIME / 2)) {
      m_ledSubsystem.putRed();
    }
    if (m_timer.advanceIfElapsed(TELEOP_BLINK_TIME)) {
      m_ledSubsystem.putBlue();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
