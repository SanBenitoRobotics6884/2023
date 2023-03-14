// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.constants.RobotConstants.Drive.*;

public class DriveDistanceCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  double m_distance;
  PIDController m_pid = new PIDController(
      DRIVE_DISTANCE_P, DRIVE_DISTANCE_I, DRIVE_DISTANCE_D);
  SlewRateLimiter m_limiter = new SlewRateLimiter(DRIVE_DISTANCE_LIMITER);
  
  /** Creates a new DriveDistanceCommand. */
  public DriveDistanceCommand(DriveSubsystem driveSubsystem, double distance) {
    m_driveSubsystem = driveSubsystem;
    m_distance = distance;
    m_pid.setTolerance(DRIVE_DISTANCE_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.setSetpoint(m_driveSubsystem.getRightDistance() + m_distance);
    m_limiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getRightDistance()),
        -DRIVE_DISTANCE_MAX_VOLTAGE, DRIVE_DISTANCE_MAX_VOLTAGE); // Might need to be negative
    output = m_limiter.calculate(output);
    m_driveSubsystem.drive(output, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}
