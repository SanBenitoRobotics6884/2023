// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.ConstantsFolder.RobotConstants.Arm.Extend.*;

/**
 * This subclasses an InstantCommand for consistency because SetArmPivotCmd has to subclass an
 * InstantCommand (finish right away). Again, there should be a WaitCommand after this command to
 * wait a time we determined before. 
 */
public class SetArmExtendCmd extends InstantCommand {
  private ArmSubsystem m_armSubsystem;
  private double m_setpoint;

  public SetArmExtendCmd(ArmSubsystem armSubsystem, double setpoint) {
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
    m_armSubsystem.setExtendSetpoint(m_setpoint);
  }
}
