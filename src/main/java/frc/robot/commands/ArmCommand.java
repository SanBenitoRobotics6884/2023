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
    double value = m_value.getAsDouble();
    double extend = m_armSubsystem.getExtendSetpoint() + Extend.Y_SCALE * value;
    double pivot = m_armSubsystem.getPivotSetpoint() + Pivot.Y_SCALE * value;
    boolean isInExtensionMode = m_extensionMode.getAsBoolean();

    if (isInExtensionMode && extend > Extend.BACK_HARD_LIMIT && extend < Extend.FRONT_HARD_LIMIT && Math.abs(value) > 0.1) {
      m_armSubsystem.setExtendSetpoint(extend);
    } else if (!isInExtensionMode && Pivot.BACK_HARD_LIMIT < pivot && pivot < Pivot.FRONT_HARD_LIMIT && Math.abs(value) > 0.1) { 
      m_armSubsystem.setPivotSetpoint(pivot);
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
