// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.ConstantsFolder.RobotConstants.Arm.*;


public class ArmCommand extends CommandBase {
  /** Creates a new ExtendCommand. */
  ArmSubsystem m_armSubsystem;
  DoubleSupplier m_value;
  BooleanSupplier m_extensionMode;
  public ArmCommand(ArmSubsystem armSubsystem, DoubleSupplier value, BooleanSupplier extensionMode) {
    m_armSubsystem = armSubsystem;
    m_value = value;
    addRequirements(armSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double extend = m_armSubsystem.getM_extendSetpoint() + Extend.Y_SCALE * m_value.getAsDouble();
    double pivot = m_armSubsystem.getM_pivotSetpoint() + Pivot.Y_SCALE * m_value.getAsDouble();

    if (m_extensionMode.getAsBoolean() && extend > Extend.BACK_HARD_LIMIT && extend < Extend.FRONT_HARD_LIMIT){
      m_armSubsystem.setM_extendSetpoint(
          m_armSubsystem.getM_extendSetpoint() + Extend.Y_SCALE * m_value.getAsDouble());
    } else if (pivot > Pivot.BACK_HARD_LIMIT && pivot < Pivot.FRONT_HARD_LIMIT) { 
      m_armSubsystem.setM_pivotSetpoint(
          m_armSubsystem.getM_pivotSetpoint() + Pivot.Y_SCALE * m_value.getAsDouble());
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
