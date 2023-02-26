// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawSubsystem;

import static frc.robot.ConstantsFolder.RobotConstants.Claw.*;

/**
 * This command extends InstantCommand so there needs to be a WaitCommand after it in autonomous)
 * Note that there are two static factory methods for the cube, cone, and open setpoint
 */
public class SetClawPositionCmd extends InstantCommand {
  private ClawSubsystem m_clawSubsystem;
  private double m_setpoint;

  public SetClawPositionCmd(ClawSubsystem clawSubsystem, double setpoint) {
    m_clawSubsystem = clawSubsystem;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawSubsystem);
  }

  public static SetClawPositionCmd createCubeCmd(ClawSubsystem clawSubsystem) {
    return new SetClawPositionCmd(clawSubsystem, CUBE_SETPOINT);
  }

  public static SetClawPositionCmd createConeCmd(ClawSubsystem clawSubsystem) {
    return new SetClawPositionCmd(clawSubsystem, CONE_SETPOINT);
  }

  public static SetClawPositionCmd createOpenCmd(ClawSubsystem clawSubsystem) {
    return new SetClawPositionCmd(clawSubsystem, OPEN_SETPOINT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_clawSubsystem.setRotations(m_setpoint);
  }
}
