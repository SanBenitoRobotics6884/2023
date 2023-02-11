// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.Arm.*;


public class ArmCommand extends CommandBase {
  /** Creates a new ExtendCommand. */
  ArmSubsystem aSubsystem;
  DoubleSupplier value;
  BooleanSupplier extensionMode;
  public ArmCommand(ArmSubsystem m_ArmSubsystem, DoubleSupplier value, BooleanSupplier extensionMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.value = value;
    this.extensionMode = extensionMode;
    addRequirements(aSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double extend = aSubsystem.getExtendSetPoint() + Extend.Y_SCALE * value.getAsDouble();
    double pivot = aSubsystem.getPivotSetPoint() + Pivot.Y_SCALE * value.getAsDouble();

    if (extensionMode.getAsBoolean() && extend > Extend.BACK_HARD_LIMIT && extend < Extend.FRONT_HARD_LIMIT){
      aSubsystem.setExtendSetPoint(
      aSubsystem.getExtendSetPoint() + Extend.Y_SCALE * value.getAsDouble());
    } else if (pivot > Pivot.BACK_HARD_LIMIT && pivot < Pivot.FRONT_HARD_LIMIT) { 
      aSubsystem.setPivotSetPoint(
        aSubsystem.getPivotSetPoint() + Pivot.Y_SCALE * value.getAsDouble());
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
