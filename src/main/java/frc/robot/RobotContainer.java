// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.RobotConstants.Claw.*;
import static frc.robot.constants.RobotConstants.Drive.*;
import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import java.util.List;

import com.fasterxml.jackson.databind.jsontype.DefaultBaseTypeLimitingValidator;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.astar.Edge;
import frc.robot.astar.Node;
import frc.robot.astar.Obstacle;
import frc.robot.astar.VisGraph;
import frc.robot.commands.AStar;
import frc.robot.commands.ClawCmmd;
import frc.robot.commands.DriveCmmd;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.Arm;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_gyro);
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(CAMERA_ONE, m_driveSubsystem);
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
 
  private final Joystick m_joystick = new Joystick(0);
  private final CommandXboxController controller = new CommandXboxController(1);
  
  private final Command m_armCommand = new ArmCommand(m_armSubsystem,
      () -> m_joystick.getY());
  private final ClawCmmd m_clawCommand = new ClawCmmd(
    m_clawSubsystem,
    () -> m_joystick.getTrigger(),
    () -> m_joystick.getRawButton(3),
    () -> m_joystick.getRawButton(4));
  private final DriveCmmd m_normalDriveCommand = new DriveCmmd(
      m_driveSubsystem,
      () -> controller.getLeftY(),
      () -> -controller.getRightX(),
      false);
  private final DriveCmmd m_snailDriveCommand = new DriveCmmd(
      m_driveSubsystem,
      () -> controller.getLeftY(),
      () -> -controller.getRightX(),
      true);

  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  private final List<Obstacle> obstacles = FieldConstants.obstacles;
  private final VisGraph AStarMap = new VisGraph();
  private final PathPlannerTrajectory trajectory;
  SequentialCommandGroup auto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_armSubsystem.setDefaultCommand(m_armCommand);
    m_clawSubsystem.setDefaultCommand(m_clawCommand);
    m_driveSubsystem.setDefaultCommand(m_normalDriveCommand);

    trajectory = PathPlanner.loadPath("Simple", CONSTRAINTS);
     auto = m_driveSubsystem.followAutoCommand(m_driveSubsystem, poseEstimatorSubsystem, trajectory);
    AStarMap.addNode(new Node(2.48 - 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 1.07 - 0.1));
    AStarMap.addNode(new Node(2.48 - 0.1, 1.07 - 0.1));
    AStarMap.addNode(new Node(3.84 + 0.1, 4.80 - 0.1)); // Divider
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node startNode = AStarMap.getNode(i);
      for (int j = i + 1; j < AStarMap.getNodeSize(); j++) {
        AStarMap.addEdge(new Edge(startNode, AStarMap.getNode(j)), obstacles);
      }
    }
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Chassis triggers
    controller.leftTrigger()
        .whileTrue(m_snailDriveCommand);
    controller.x().whileTrue(new AStar(
        m_driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), 
        Rotation2d.fromDegrees(180)), obstacles, AStarMap));
    controller.y().whileTrue(new AStar(
        m_driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), 
        Rotation2d.fromDegrees(180)), obstacles, AStarMap));
    controller.a().onTrue(new InstantCommand(m_driveSubsystem::resetEncoders));
    
    // Claw triggers
    new JoystickButton(m_joystick, 2)
        .onTrue(new InstantCommand(m_clawSubsystem::colorCheck)); // To close the claw (with color sensor) 
    new JoystickButton(m_joystick, 1).negate()
        .and(new JoystickButton(m_joystick, 3))
        .and(new JoystickButton(m_joystick, 4))
        .onTrue(new InstantCommand(() -> m_clawSubsystem.setRotations(OPEN_SETPOINT))); 

    // Arm triggers
    new JoystickButton(m_joystick, 11)
         .onTrue(new InstantCommand(() -> m_armSubsystem.setExtendSetpoint(Arm.Extend.HYBRID_SETPOINT))); 

    new JoystickButton(m_joystick, 9)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setExtendSetpoint(Arm.Extend.MID_SETPOINT)));

    new JoystickButton(m_joystick, 7)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setExtendSetpoint(Arm.Extend.HIGH_SETPOINT)));

    new JoystickButton(m_joystick, 12)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setPivotSetpoint(Arm.Pivot.HYBRID_SETPOINT)));
    
    new JoystickButton(m_joystick, 10)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setPivotSetpoint(Arm.Pivot.MID_SETPOINT)));
    
    new JoystickButton(m_joystick, 8)
        .onTrue(new InstantCommand(() -> m_armSubsystem.setPivotSetpoint(Arm.Pivot.HIGH_SETPOINT)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*return new SequentialCommandGroup(
        new PPRamseteCommand(
            trajectory, 
            poseEstimatorSubsystem::getPose2d, 
            RAMSETE_CONTROLLER, 
            FEED_FOWARD, 
            KINEMATICS, 
            m_driveSubsystem::getWheelSpeeds, 
            LEFT_DRIVE_CONTROLLER, 
            RIGHT_DRIVE_CONTROLLER, 
            m_driveSubsystem::tankDrive, 
            false, 
            m_driveSubsystem),
        new RunCommand(m_driveSubsystem::stopMotors, m_driveSubsystem));
        */
        return auto;
  }
}
