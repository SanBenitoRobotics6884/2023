// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.RobotConstants.Claw.*;
import static frc.robot.constants.RobotConstants.Drive.*;
import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

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
import frc.robot.util.JoystickMultiPress;
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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(m_gyro);


  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(CAMERA_ONE, driveSubsystem);
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
 
  Joystick m_joystick = new Joystick(0);
  Trigger m_joystickTrigger = new JoystickButton(m_joystick, 1);
  Trigger m_joystickCloseClawLeft = new JoystickButton(m_joystick, 3);
  Trigger m_joystickOpenClawRight = new JoystickButton(m_joystick, 4);
  private final Command m_armCommand = new ArmCommand(m_armSubsystem,
    () -> -m_joystick.getY());

  CommandXboxController controller = new CommandXboxController(0);

  VisGraph AStarMap = new VisGraph();
  

  ClawCmmd clawCommand = new ClawCmmd(
    clawSubsystem,
    () -> m_joystick.getTrigger(),
    () -> m_joystick.getRawButton(3),
    () -> m_joystick.getRawButton(4));

  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  final List<Obstacle> obstacles = FieldConstants.obstacles;

  PathPlannerTrajectory trajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     trajectory = PathPlanner.loadPath("Simple", CONSTRAINTS );
    
    m_armSubsystem.setDefaultCommand(m_armCommand);
    driveSubsystem.setDefaultCommand(new DriveCmmd(driveSubsystem,
     ()->controller.getRightY(), ()->controller.getRightX(), false));
   
     configureButtonBindings();

    AStarMap.addNode(new Node(2.48 - 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 1.07 - 0.1));
    AStarMap.addNode(new Node(2.48 - 0.1, 1.07 - 0.1));
    // Divider
    AStarMap.addNode(new Node(3.84 + 0.1, 4.80 - 0.1));

    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node startNode = AStarMap.getNode(i);
      for (int j = i + 1; j < AStarMap.getNodeSize(); j++) {
        AStarMap.addEdge(new Edge(startNode, AStarMap.getNode(j)), obstacles);
      }
    }

    clawSubsystem.setDefaultCommand(clawCommand);
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
        .whileTrue(new DriveCmmd(
            driveSubsystem, 
            ()->controller.getLeftY(), 
            ()->controller.getRightX(), 
            true));
    controller.x().whileTrue(new AStar(
        driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), Rotation2d.fromDegrees(180)), obstacles,
        AStarMap));
    controller.y().whileTrue(new AStar(
        driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), Rotation2d.fromDegrees(180)), obstacles,
        AStarMap));
    controller.a().whileTrue(new RunCommand(driveSubsystem::ResetEncoder, driveSubsystem) );
    
    // Claw triggers
    new JoystickButton(m_joystick, 2)
        .onTrue(new InstantCommand(() -> clawSubsystem.colorCheck())); // To close the claw (with color sensor) 
    new JoystickMultiPress(m_joystick, 3)
        .and(m_joystickTrigger.negate())
        .onTrue(new InstantCommand(() -> clawSubsystem.setRotations(OPEN_SETPOINT))); 
    new JoystickMultiPress(m_joystick, 4)
        .and(m_joystickTrigger.negate())
        .onTrue(new InstantCommand(() -> clawSubsystem.setRotations(OPEN_SETPOINT)));  

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

    new Trigger(() -> m_joystick.getPOV() == 0)
        .onTrue(new InstantCommand(() -> m_armSubsystem.resetEncoders()));

    new Trigger(() -> m_joystick.getPOV() == 180)
        .onTrue(new InstantCommand(() -> m_armSubsystem.printCANCoderOffset()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
        new PPRamseteCommand(
            trajectory, 
            poseEstimatorSubsystem::getPose2d, 
            RAMSETE_CONTROLLER, 
            FEED_FOWARD, 
            KINEMATICS, 
            driveSubsystem::getWheelSpeeds, 
            LEFT_DRIVE_CONTROLLER, 
            RIGHT_DRIVE_CONTROLLER, 
            driveSubsystem::SetMotorVoltage, 
            false, 
            driveSubsystem),
        new RunCommand(driveSubsystem::StopMotors, driveSubsystem));
  }
}
