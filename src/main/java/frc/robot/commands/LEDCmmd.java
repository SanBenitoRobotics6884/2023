// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.BooleanChecker;

public class LEDCmmd extends CommandBase {
  public LEDSubsystem m_LEDSubsystem;
  double startTime;
  double currentTime;
  boolean isOff = true;
  BooleanChecker checker = new BooleanChecker(() -> (Timer.getFPGATimestamp() - startTime) % 0.5 > 0.25);

  public LEDCmmd(LEDSubsystem ledSubsystem) {
    m_LEDSubsystem = ledSubsystem;
    addRequirements(m_LEDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();

    if (checker.check()) {
      if (isOff) {
        m_LEDSubsystem.putRed();
        isOff = false;
      } else {
        m_LEDSubsystem.putOff();
        isOff = true;
      }
    }
  } // Remember to % (modulo)

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
