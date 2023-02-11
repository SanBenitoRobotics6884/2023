// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Drive.*;
import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectorySubystem extends SubsystemBase {
  List<PathPoint> m_BlueGridOnePoints;
  List<PathPoint> m_BlueGridTwoPoints;
  List<PathPoint> m_BlueGridThreePoints;
  List<PathPoint> m_BlueLoadingPoints;
  List<PathPoint> m_RedGridOnePoints;
  List<PathPoint> m_RedGridTwoPoints;
  List<PathPoint> m_RedGridThreePoints;
  List<PathPoint> m_RedLoadingPoints;

  DriveSubsystem driveSubsystem;
  PoseEstimatorSubsystem poseEstimator;

  
  public TrajectorySubystem(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    new PathPoint(null, null);
   
    this.m_BlueGridOnePoints = Collections.unmodifiableList
    (List.of(new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation()),
      new PathPoint(TAG_EIGHT_POSE.toPose2d().getTranslation(), TAG_EIGHT_POSE.toPose2d().getRotation())));
   
    this.m_BlueGridTwoPoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation()),
      new PathPoint(TAG_SEVEN_POSE.toPose2d().getTranslation(), TAG_SEVEN_POSE.toPose2d().getRotation()) ));
   
    this.m_BlueGridThreePoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
      ,new PathPoint(TAG_SIX_POSE.toPose2d().getTranslation(), TAG_SIX_POSE.toPose2d().getRotation())));
   
    this.m_BlueLoadingPoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
      ,new PathPoint(TAG_FOUR_POSE.toPose2d().getTranslation(), TAG_FOUR_POSE.toPose2d().getRotation())));
   
    this.m_RedGridOnePoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
      ,new PathPoint(TAG_THREE_POSE.toPose2d().getTranslation(), TAG_THREE_POSE.toPose2d().getRotation())));
   
    this.m_RedGridTwoPoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
      ,new PathPoint(TAG_TWO_POSE.toPose2d().getTranslation(), TAG_TWO_POSE.toPose2d().getRotation())));
   
    this.m_RedGridThreePoints = Collections.unmodifiableList(List.of(
      new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
      ,new PathPoint(TAG_ONE_POSE.toPose2d().getTranslation(), TAG_ONE_POSE.toPose2d().getRotation())));
    
      this.m_RedLoadingPoints = Collections.unmodifiableList(List.of(
        new PathPoint( poseEstimator.getPose2d().getTranslation(), poseEstimator.getPose2d().getRotation())
        ,new PathPoint(TAG_FIVE_POSE.toPose2d().getTranslation(), TAG_FIVE_POSE.toPose2d().getRotation())));  


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Trajectory GetBlueGridOneTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_BlueGridOnePoints);
    return trajectory;
  }
  public Trajectory GetBlueGridTwoTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_BlueGridTwoPoints);
    return trajectory;
  }
  public Trajectory GetBlueGridThreeTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_BlueGridThreePoints);
    return trajectory;
  }
  public Trajectory GetBlueLoadingTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_BlueLoadingPoints);
    return trajectory;
  }
  public Trajectory GetRedGridOneTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_RedGridOnePoints);
    return trajectory;
  }
  public Trajectory GetRedGridTwoTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_RedGridTwoPoints);
    return trajectory;
  }
  public PathPlannerTrajectory GetRedGridThreeTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_RedGridThreePoints);
    return trajectory;
  }
  public PathPlannerTrajectory GetRedLoadingTrajectory(){
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(CONSTRAINTS, m_RedLoadingPoints);
    return trajectory;
  }

  public Trajectory getGridOneTrajectory(){
    if( DriverStation.getAlliance() == Alliance.Blue){
      return GetBlueGridOneTrajectory();
    }
    else{
      return GetRedGridOneTrajectory();
      }
    }
    public Trajectory getGridTwoTrajectory(){
      if( DriverStation.getAlliance() == Alliance.Blue){
        return GetBlueGridTwoTrajectory();
      }
      else{
        return GetRedGridTwoTrajectory();
      }

  }
  public Trajectory getGridThreeTrajectory(){
    if( DriverStation.getAlliance() == Alliance.Blue){
      return GetBlueGridThreeTrajectory();
    }
    else{
      return GetRedGridThreeTrajectory();
    }

    }
    public Trajectory getLoadingZoneTrajectory(){
      if( DriverStation.getAlliance() == Alliance.Blue){
        return GetBlueLoadingTrajectory();
      }
      else{
        return GetRedLoadingTrajectory();
      }
   
   
 }
}
