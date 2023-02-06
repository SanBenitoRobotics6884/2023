// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.ConstantsFolder.RobotConstants.FiducialTracking.*;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.AStar.Edge;
import frc.robot.AStar.Node;
import frc.robot.AStar.Obstacle;
import frc.robot.AStar.VisGraph;
import frc.robot.ConstantsFolder.FieldConstants;
import frc.robot.commands.AStar;
import frc.robot.commands.DriveCmmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TrajectorySubystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(CAMERA_ONE, driveSubsystem);
  private final TrajectorySubystem trajectorySubystem = new TrajectorySubystem(driveSubsystem, poseEstimatorSubsystem);
  Joystick joystick = new Joystick(0);
  JoystickButton aButton = new JoystickButton(joystick, 0);

CommandXboxController controller = new CommandXboxController(0);

  VisGraph AStarMap = new VisGraph();

  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  final List<Obstacle> obstacles = FieldConstants.obstacles;


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    driveSubsystem.setDefaultCommand(new DriveCmmd(driveSubsystem,
     ()->controller.getLeftY(), ()->controller.getRightX(), false));
   
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

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   controller.leftTrigger().whileTrue(new DriveCmmd(driveSubsystem, ()->controller.getLeftY(), ()->controller.getRightX(), true));

      controller.x().whileTrue(new AStar(
        driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), Rotation2d.fromDegrees(180)), obstacles,
        AStarMap));

    controller.y().whileTrue(new AStar(
        driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), Rotation2d.fromDegrees(180)), obstacles,
        AStarMap));
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
