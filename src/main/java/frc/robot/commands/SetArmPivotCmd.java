// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.RobotConstants.Arm.Pivot.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Until we add a feedforward for gravity (which will require us to be able to know the exact
 * position of the arm pivot and then run sysid on the arm) in autonomous there should just be a
 * WaitCommand that waits a time that we can determine while testing. If we get that going, this
 * won't have to subclass InstantCommand and it can be a command that finishes after the position 
 * of the arm and the setpoint are within a certain error. (And we can remove the WaitCommand)
 */
public class SetArmPivotCmd extends InstantCommand {
  private ArmSubsystem m_armSubsystem;
  private double m_setpoint;

  public SetArmPivotCmd(ArmSubsystem armSubsystem, double setpoint) {
    m_armSubsystem = armSubsystem;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  public static SetArmExtendCmd createHybridCmd(ArmSubsystem armSubsystem) {
    return new SetArmExtendCmd(armSubsystem, HYBRID_SETPOINT);
  }

  public static SetArmExtendCmd createMidCmd(ArmSubsystem armSubsystem) {
    return new SetArmExtendCmd(armSubsystem, MID_SETPOINT);
  }

  public static SetArmExtendCmd createHighCmd(ArmSubsystem armSubsystem) {
    return new SetArmExtendCmd(armSubsystem, HIGH_SETPOINT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setPivotSetpoint(m_setpoint);
  }
}
