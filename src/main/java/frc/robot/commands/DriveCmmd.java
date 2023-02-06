// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCmmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final DoubleSupplier fowDoubleSupplier;
  private final DoubleSupplier m_thetaDoubleSupplier;
  private Boolean m_turbo;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmmd(DriveSubsystem subsystem, DoubleSupplier f, DoubleSupplier t, Boolean turbo) {
    m_subsystem = subsystem;
    fowDoubleSupplier = f;
    m_thetaDoubleSupplier = t;
    m_turbo = turbo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_turbo){
      m_subsystem.TurboJoystickDrive(fowDoubleSupplier.getAsDouble(), m_thetaDoubleSupplier.getAsDouble());
    }
    if(!m_turbo){
      m_subsystem.NormalDrive(fowDoubleSupplier.getAsDouble(), m_thetaDoubleSupplier.getAsDouble());
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
