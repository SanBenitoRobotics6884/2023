// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.Drive.*;
import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new poseEstimator. */
  DifferentialDrivePoseEstimator m_poseEstimator;
  double m_previousTimestamp = 0.0;
  DriveSubsystem m_driveSubsystem;
  PhotonCamera m_camera;
  Field2d m_field2d;
  AprilTagFieldLayout m_layout;
  Optional<EstimatedRobotPose> m_photonEstimatedPose = Optional.empty();
  PhotonPoseEstimator m_photonPoseEstimator;
  public PoseEstimatorSubsystem(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.m_camera = camera;
    this.m_driveSubsystem = driveSubsystem;
    new Rotation2d();
    m_poseEstimator = new DifferentialDrivePoseEstimator(KINEMATICS, driveSubsystem.getRotation2D() 
    , driveSubsystem.getLeftDistance(), driveSubsystem.getRightDistance(), new Pose2d(),
    //State Standard Deviations X, Y, Theta
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, Units.degreesToRadians(5)), 
     //Vision Measurement Standard Deviations X, Y, Theta
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02,Units.degreesToRadians(5) ));
    try{
     m_layout =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
     var alliance = DriverStation.getAlliance();
    m_layout.setOrigin(alliance==Alliance.Blue? OriginPosition.kBlueAllianceWallRightSide :
     OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e){
      DriverStation.reportError("could not load layout ", e.getStackTrace());
      m_layout = null;

    }
     
     m_photonPoseEstimator = new PhotonPoseEstimator(m_layout, PoseStrategy.LOWEST_AMBIGUITY, camera, BOT_TO_CAMERA);

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");


     

     m_field2d = new Field2d();
     tab.add("field", m_field2d).withPosition(2, 0).withSize(6, 4);
     tab.addString("estimatedPose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_poseEstimator.update(m_driveSubsystem.getRotation2D(), m_driveSubsystem.getLeftDistance(), m_driveSubsystem.getRightDistance());

    m_photonEstimatedPose = m_photonPoseEstimator.update();

    if(m_photonEstimatedPose.isPresent()){
      EstimatedRobotPose pose = m_photonEstimatedPose.get();
      if(Math.hypot(pose.estimatedPose.getX(), pose.estimatedPose.getY() )> 5.75){
        try{
        m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        } catch (ConcurrentModificationException e){

        }
      }
    }
    m_field2d.setRobotPose(getPose2d());
    if(DriverStation.getAlliance() == Alliance.Red){
      m_field2d.setRobotPose(FIELD_LENGTH- getPose2d().getX(), FIELD_WIDTH - getPose2d().getY(),
       new Rotation2d(getPose2d().getRotation().getRadians() + Math.PI));
    }

  }
 
  public Pose2d getPose2d(){
   return m_poseEstimator.getEstimatedPosition();
  }
  public void ResetPose2d(Pose2d pose2d){
    m_poseEstimator.resetPosition(
      m_driveSubsystem.getRotation2D(), m_driveSubsystem.getLeftDistance(), m_driveSubsystem.getRightDistance(), pose2d);
      
  }
  public void AddTrajectory(PathPlannerTrajectory trajectory){
    m_field2d.getObject("traj").setTrajectory(trajectory);
  }
  private String getFomattedPose() {
    var pose = getPose2d();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
 
}
