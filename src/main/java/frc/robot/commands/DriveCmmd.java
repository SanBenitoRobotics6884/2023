// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.constants.RobotConstants.Drive.*;

/** An example command that uses an example subsystem. */
public class DriveCmmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final SlewRateLimiter m_forwLimiter;
  private final SlewRateLimiter m_rotLimiter;
  private final DoubleSupplier m_forwSupplier;
  private final DoubleSupplier m_rotSupplier;
  private final Boolean m_snailMode;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmmd(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, Boolean inSnailMode) {
    m_subsystem = subsystem;
    m_forwLimiter = new SlewRateLimiter(FORWARD_RATE_LIMIT);
    m_rotLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
    m_forwSupplier = forward;
    m_rotSupplier = rotation;
    m_snailMode = inSnailMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forw = m_forwLimiter.calculate(m_forwSupplier.getAsDouble());
    double rot = m_rotLimiter.calculate(m_rotSupplier.getAsDouble());
    if(m_snailMode){
      m_subsystem.snailDrive(forw, rot);
    } else {
      m_subsystem.drive(forw, rot);
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
