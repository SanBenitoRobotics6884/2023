// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.FiducialTracking.*;

public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new poseEstimator. */
  DifferentialDrivePoseEstimator poseEstimator;
  double previousTimeStamp = 0.0;
  DriveSubsystem driveSubystem;
  PhotonCamera camera;

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
    
     
    



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var result = camera.getLatestResult();
    var resultTimeStamp = result.getTimestampSeconds();
    
    if(result.hasTargets() && resultTimeStamp!= previousTimeStamp){
      var target = result.getBestTarget();
      var previousTimeStamp = resultTimeStamp;
      
      
      if(target.getPoseAmbiguity() <= .2 && target.getFiducialId()>= 6 || target.getFiducialId() == 4
       && ALLIANCE_COLOR == AllianceColor.BLUE_ALLIANCE ){
        TAG_POSES.setOrigin(TAG_EIGHT_POSE);;
        var targetId = target.getFiducialId();
        Pose3d targetPose = TAG_POSES.getTagPose(targetId).get();
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());  

        Pose3d Robotpose = cameraPose.transformBy(CAMERA_TO_ROBOT);

        poseEstimator.addVisionMeasurement(Robotpose.toPose2d(), resultTimeStamp);
      }
      
   
      else if(target.getPoseAmbiguity() <= .2 && target.getFiducialId()<= 4 || target.getFiducialId() == 5
       && ALLIANCE_COLOR == AllianceColor.RED_ALLIANCE ){
        var targetId = target.getFiducialId();
        Pose3d targetPose = TAG_POSES.getTagPose(targetId).get();
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());  

        Pose3d Robotpose = cameraPose.transformBy(CAMERA_TO_ROBOT);

        poseEstimator.addVisionMeasurement(Robotpose.toPose2d(), resultTimeStamp);



        

      }
      poseEstimator.update(driveSubystem.getRotation2D(), driveSubystem.getLeftDistance(), driveSubystem.getRightDistance());
    }
      

  }
  public void setBlueAllianceColor(){
    ALLIANCE_COLOR = AllianceColor.BLUE_ALLIANCE;
  }

  public void setRedAllianceColor(){
    ALLIANCE_COLOR = AllianceColor.RED_ALLIANCE;
  }
  public Pose2d getPose2d(){
   return poseEstimator.getEstimatedPosition();
  }
  public void ResetPose2d(Pose2d pose2d){
    poseEstimator.resetPosition(
      driveSubystem.getRotation2D(), driveSubystem.getLeftDistance(), driveSubystem.getRightDistance(), pose2d);
      
  }
  public void AutoBalance(){
    
  }
 
}
