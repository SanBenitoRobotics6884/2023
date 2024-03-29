// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.astar.Edge;
import frc.robot.astar.Node;
import frc.robot.astar.Obstacle;
import frc.robot.astar.VisGraph;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AStar extends CommandBase {
  private final DriveSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private PPRamseteCommand pathDrivingCommand;
  private final PathConstraints constraints;
  private final Node finalPosition;
  private Node startPoint;
  private final List<Obstacle> obstacles;
  private VisGraph AStarMap;

  public AStar(DriveSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, Node finalPosition,
      List<Obstacle> obstacles, VisGraph AStarMap) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.obstacles = obstacles;
    this.finalPosition = finalPosition;
    this.AStarMap = AStarMap;
    this.startPoint = new Node(p);
    AStarMap.addNode(finalPosition);
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node endNode = AStarMap.getNode(i);
      AStarMap.addEdge(new Edge(finalPosition, endNode), obstacles);
    }
    addRequirements(driveSystem, poseEstimatorSystem);
  }

  // ----------------------------------------------------------------------------
  // Pre-schedule setup code.
  @Override
  public void initialize() {
    VisGraph tempGraph = AStarMap;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      startPoint = new Node(poseEstimatorSystem);
    } else {
      Pose2d flippedY = new Pose2d(poseEstimatorSystem.getPose2d().getX(),
           FieldConstants.FIELD_WIDTH - poseEstimatorSystem.getPose2d().getY(),
          poseEstimatorSystem.getPose2d().getRotation());
      startPoint = new Node(flippedY);
    }
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    tempGraph.addNode(startPoint);
    if (tempGraph.addEdge(new Edge(startPoint, finalPosition), obstacles)) {
      fullPath.add(0, startPoint);
      fullPath.add(1, finalPosition);
    } else {
      for (int i = 0; i < tempGraph.getNodeSize(); i++) {
        Node endNode = tempGraph.getNode(i);
        tempGraph.addEdge(new Edge(startPoint, endNode), obstacles);
      }
      fullPath = tempGraph.findPath(startPoint, finalPosition);
    }

    if (fullPath == null) {
      return;
    }

    // Gets speed of robot
    double startingSpeed = Math.hypot(driveSystem.getChassisSpeeds().vxMetersPerSecond,
        driveSystem.getChassisSpeeds().vyMetersPerSecond);
    Rotation2d heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(),
        fullPath.get(1).getY() - startPoint.getY());

    // If the robot is moving over a specified speed take movement into account.
    if (startingSpeed > 0.05) {
      heading = new Rotation2d(driveSystem.getChassisSpeeds().vxMetersPerSecond,
          driveSystem.getChassisSpeeds().vyMetersPerSecond);
    }

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];

    // Find path between points
    for (int i = 0; i < fullPath.size(); i++) {
      if (i == 0) {
        fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
            startPoint.getHolRot(), startingSpeed);
      } else if (i + 1 == fullPath.size()) {
        fullPathPoints[i] = new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
            new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
                fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                finalPosition.getHolRot());
      } else {
        // Change allianceFinal.getHolRot() to null if you want it to turn smoothly over
        // path. (Needs more testing)
        fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
            new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
                fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
                finalPosition.getHolRot());
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    System.out.println(fullPathPoints);
    trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
    // Change trajectory based on alliance color
    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
    // Display Trajectory
    poseEstimatorSystem.AddTrajectory(trajectory);
    pathDrivingCommand = DriveSubsystem.followTrajCommand(driveSystem, poseEstimatorSystem, trajectory);
    pathDrivingCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return (pathDrivingCommand == null || !pathDrivingCommand.isScheduled());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pathDrivingCommand.cancel();
    }

    driveSystem.stopMotors();
  }
}