// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.RobotConstants.Arm.Extend.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/**
 * This subclasses an InstantCommand for consistency because SetArmPivotCmd has to subclass an
 * InstantCommand (finish right away). Again, there should be a WaitCommand after this command to
 * wait a time we determined before. 
 */
public class SetArmExtendCmd extends InstantCommand {
  private ExtendSubsystem m_extendSubsystem;
  private double m_setpoint;

  public SetArmExtendCmd(PivotSubsystem pivotSubsystem, double setpoint) {
    m_extendSubsystem = extendSubsystem;
    m_setpoint = setpoint;
    ExtendSubsystem extendSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extendSubsystem);
  }

  public static SetArmExtendCmd createHybridCmd(ExtendSubsystem extendSubsystem) {
    return new SetArmExtendCmd(extendSubsystem, HYBRID_SETPOINT);
  }

  public static SetArmExtendCmd createMidCmd(ExtendSubsystem extendSubsystem) {
    return new SetArmExtendCmd(extendSubsystem, MID_SETPOINT);
  }

  public static SetArmExtendCmd createHighCmd(ExtendSubsystem extendSubsystem) {
    return new SetArmExtendCmd(extendSubsystem, HIGH_SETPOINT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extendSubsystem.setExtendSetpoint(m_setpoint);
  }
}
