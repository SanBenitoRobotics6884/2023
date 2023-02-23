// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ConstantsFolder.RobotConstants.Drive.*;
import static frc.robot.ConstantsFolder.RobotConstants.FiducialTracking.*;

import java.io.IOException;
import java.sql.Driver;
import java.util.Collections;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Optional;

import javax.lang.model.util.Elements.Origin;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsFolder.FieldConstants;


public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new poseEstimator. */
  DifferentialDrivePoseEstimator poseEstimator;
  double previousTimeStamp = 0.0;
  DriveSubsystem driveSubystem;
  PhotonCamera camera;
  Field2d m_Field2d;
  AprilTagFieldLayout layout;
  Optional<EstimatedRobotPose> photonEstimatedPose = Optional.empty();
  PhotonPoseEstimator photonPoseEstimator;
  public PoseEstimatorSubsystem(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    this.camera = camera;
    this.driveSubystem = driveSubsystem;
    new Rotation2d();
    poseEstimator = new DifferentialDrivePoseEstimator(KINEMATICS, driveSubsystem.getRotation2D() 
    , driveSubsystem.getLeftDistance(), driveSubsystem.getRightDistance(), new Pose2d(),
    //State Standard Deviations X, Y, Theta
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, Units.degreesToRadians(5)), 
     //Vision Measurement Standard Deviations X, Y, Theta
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02,Units.degreesToRadians(5) ));
    try{
     layout =  AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
     var alliance = DriverStation.getAlliance();
    layout.setOrigin(alliance==Alliance.Blue? OriginPosition.kBlueAllianceWallRightSide :
     OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e){
      DriverStation.reportError("could not load layout ", e.getStackTrace());
      layout = null;

    }
     
     photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, camera, BOT_TO_CAMERA);

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");


     

     m_Field2d = new Field2d();
     tab.add("field", m_Field2d).withPosition(2, 0).withSize(6, 4);
     tab.addString("estimatedPose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    photonEstimatedPose = photonPoseEstimator.update();

    if(photonEstimatedPose.isPresent()){
      EstimatedRobotPose pose = photonEstimatedPose.get();
      if(Math.hypot(pose.estimatedPose.getX(), pose.estimatedPose.getY() )> 5.75){
        try{
        poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        } catch (ConcurrentModificationException e){

        }
      }
    }
    m_Field2d.setRobotPose(getPose2d());
    if(DriverStation.getAlliance() == Alliance.Red){
      m_Field2d.setRobotPose(FIELD_LENGTH- getPose2d().getX(), FIELD_WIDTH - getPose2d().getY(),
       new Rotation2d(getPose2d().getRotation().getRadians() + Math.PI));
    }

  }
 
  public Pose2d getPose2d(){
   return poseEstimator.getEstimatedPosition();
  }
  public void ResetPose2d(Pose2d pose2d){
    poseEstimator.resetPosition(
      driveSubystem.getRotation2D(), driveSubystem.getLeftDistance(), driveSubystem.getRightDistance(), pose2d);
      
  }
  public void AddTrajectory(PathPlannerTrajectory trajectory){
    m_Field2d.getObject("traj").setTrajectory(trajectory);
  }
  private String getFomattedPose() {
    var pose = getPose2d();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
 
}
