// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import static frc.robot.constants.RobotConstants.LED.*;

public class ChaseLEDCommand extends CommandBase {
  LEDSubsystem m_ledSubsystem;
  Timer m_timer;
  int m_startPos = 0;
  public ChaseLEDCommand(LEDSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ledSubsystem = subsystem;
    addRequirements(m_ledSubsystem);
  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_ledSubsystem.setIndividualMode();
    for (int i = 0; i < LENGTH; i++){
      m_ledSubsystem.setLED(i, RED);
    }
    for (int i = 0; i < CHASE_LENGTH; i++) {
      m_ledSubsystem.setLED(i, BLUE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.advanceIfElapsed(CHASE_TIME)) {
      m_ledSubsystem.setLED(m_startPos, RED);
      m_ledSubsystem.setLED(m_startPos + CHASE_LENGTH, BLUE);
      m_startPos++;
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
