// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateGyro extends CommandBase {
  /** Creates a new CalibrateGyro. */
  DriveSubsystem driveSubsystem;
  public CalibrateGyro(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.calibrateGyro();
  }

 
  @Override
  public boolean isFinished() {
    return true;
  }
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
