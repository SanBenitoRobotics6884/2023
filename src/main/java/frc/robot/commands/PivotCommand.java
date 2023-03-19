// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.RobotConstants.Pivot.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;


public class PivotCommand extends CommandBase {
  /** Creates a new ExtendCommand. */
  PivotSubsystem m_pivotSubsystem;
  DoubleSupplier m_value;
  BooleanSupplier m_extensionMode;
  public PivotCommand(PivotSubsystem pivotSubsystem, DoubleSupplier value) {
    m_pivotSubsystem = pivotSubsystem;
    m_value = value;
    addRequirements(pivotSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = m_value.getAsDouble();
    double pivot = m_pivotSubsystem.getSetpoint() + Y_SCALE * value;
    if (BACK_HARD_LIMIT <= pivot && pivot <= FRONT_HARD_LIMIT && Math.abs(value) > 0.1) { 
      m_pivotSubsystem.setSetpoint(pivot);
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
