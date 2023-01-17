// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.FiducialTracking.*;

public class TrajectorySubystem extends SubsystemBase {
  List<PathPoint> m_BlueGridOnePoints;
  List<PathPoint> m_BlueGridTwoPoints;
  List<PathPoint> m_BlueGridThreePoints;
  List<PathPoint> m_RedGridOnePoints;
  List<PathPoint> m_RedGridTwoPoints;
  List<PathPoint> m_RedGridThreePoints;

  DriveSubsystem driveSubsystem;
  PoseEstimatorSubsystem poseEstimator;

  
  public TrajectorySubystem(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimator) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimator = poseEstimator;
    new PathPoint(null, null);
    this.m_BlueGridOnePoints = Collections.unmodifiableList
    (List.of(PathPoint.fromCurrentDifferentialState(poseEstimator.getPose2d(),driveSubsystem.getChassisSpeeds() )));
    this.m_BlueGridTwoPoints = Collections.unmodifiableList(List.of());
    this.m_BlueGridThreePoints = Collections.unmodifiableList(List.of());
    this.m_RedGridOnePoints = Collections.unmodifiableList(List.of());
    this.m_RedGridTwoPoints = Collections.unmodifiableList(List.of());
    this.m_RedGridThreePoints = Collections.unmodifiableList(List.of());



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Trajectory GetBlueGridOneTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_EIGHT_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetBlueGridTwoTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_SEVEN_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetBlueGridThreeTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_THREE_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetBlueLoadingTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_FOUR_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetRedGridOneTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_ONE_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetRedGridTwoTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_TWO_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetRedGridThreeTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_THREE_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }
  public Trajectory GetRedLoadingTrajectory(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      poseEstimator.getPose2d(), null,TAG_FIVE_POSE.toPose2d() , TRAJECTORY_CONFIG);
    return trajectory;
  }

  public Trajectory getGridOneTrajectory(){
    if(ALLIANCE_COLOR == AllianceColor.BLUE_ALLIANCE){
      return GetBlueGridOneTrajectory();
    }
    else{
      return GetRedGridOneTrajectory();
      }
    }
    public Trajectory getGridTwoTrajectory(){
      if(ALLIANCE_COLOR == AllianceColor.BLUE_ALLIANCE){
        return GetBlueGridTwoTrajectory();
      }
      else{
        return GetRedGridTwoTrajectory();
      }

  }
  public Trajectory getGridThreeTrajectory(){
    if(ALLIANCE_COLOR == AllianceColor.BLUE_ALLIANCE){
      return GetBlueGridThreeTrajectory();
    }
    else{
      return GetRedGridThreeTrajectory();
    }

    }
    public Trajectory getLoadingZoneTrajectory(){
      if(ALLIANCE_COLOR == AllianceColor.BLUE_ALLIANCE){
        return GetBlueLoadingTrajectory();
      }
      else{
        return GetRedLoadingTrajectory();
      }
        
      }
 }
