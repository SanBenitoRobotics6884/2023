// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.RobotConstants.Arm.Pivot.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PivotSubsystem;

/**
 * Until we add a feedforward for gravity (which will require us to be able to know the exact
 * position of the arm pivot and then run sysid on the arm) in autonomous there should just be a
 * WaitCommand that waits a time that we can determine while testing. If we get that going, this
 * won't have to subclass InstantCommand and it can be a command that finishes after the position 
 * of the arm and the setpoint are within a certain error. (And we can remove the WaitCommand)
 */
public class SetArmPivotCmd extends InstantCommand {
  private PivotSubsystem m_pivotSubsystem;
  private double m_setpoint;

  public SetArmPivotCmd(PivotSubsystem pivotSubsystem, double setpoint) {
    m_pivotSubsystem = pivotSubsystem;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivotSubsystem);
  }

  public static SetArmExtendCmd createHybridCmd(PivotSubsystem pivotSubsystem) {
    return new SetArmPivotCmd(pivotSubsystem, HYBRID_SETPOINT);
  }

  public static SetArmExtendCmd createMidCmd(PivotSubsystem pivotSubsystem) {
    return new SetArmPivotCmd(pivotSubsystem, MID_SETPOINT);
  }

  public static SetArmExtendCmd createHighCmd(PivotSubsystem pivotSubsystem) {
    return new SetArmPivotCmdSubsystem(pivotSubsystem, HIGH_SETPOINT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotSubsystem.setPivotSetpoint(m_setpoint);
  }
}
