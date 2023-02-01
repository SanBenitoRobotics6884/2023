// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.ConstantsFolder.RobotConstants.FiducialTracking.*;

import java.util.Optional;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  double m_yaw;
  double m_pitch;
  double m_distance;
  int m_fiducialID;
  PhotonCamera m_camera;
  DriveSubsystem m_driveSubsystem;
  double m_targetHeight;
  Pose3d m_tagPose3d;
  Transform3d cameraToTarget;
  
  public VisionSubsystem(PhotonCamera camera, DriveSubsystem driveSubsystem) {
    m_camera = camera;
    m_driveSubsystem = driveSubsystem;
    m_yaw = 0.0;
    m_fiducialID = 0;
    m_targetHeight = 0.0;
    m_tagPose3d = new Pose3d();
  }


  @Override
  public void periodic() {
    var result = m_camera.getLatestResult();
    if(result.hasTargets()){
      var target = result.getBestTarget();
      if(target.getPoseAmbiguity()<= .2){
        m_fiducialID = target.getFiducialId();
        m_yaw = target.getYaw();
        m_pitch = target.getPitch();
        m_tagPose3d = TAG_POSES.getTagPose(m_fiducialID).get();
       m_distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_ONE_HEIGHT, m_tagPose3d.getZ(), CAMERA_ONE_PITCH_RADIANS, GetPitchRadians());
         cameraToTarget = target.getBestCameraToTarget();
        
        
      }
    }
  }
  public double GetYawDegrees(){
    return m_yaw;
  }
  public double GetYawRadians(){
    return m_yaw * Math.PI /180;
  }
  public double GetPitchDegrees(){
    return m_pitch;
  }
  public double GetPitchRadians(){
    return m_pitch * Math.PI /180;
  }

}
