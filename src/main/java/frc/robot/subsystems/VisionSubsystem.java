// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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
      double ambiguity = target.getPoseAmbiguity();
      if(ambiguity < 0.2 && ambiguity != -1){
        m_fiducialID = target.getFiducialId();
        m_yaw = target.getYaw();
        m_pitch = target.getPitch();
        m_tagPose3d = TAG_POSES.getTagPose(m_fiducialID).get();
        m_distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_ONE_HEIGHT, m_tagPose3d.getZ(), CAMERA_ONE_PITCH_RADIANS, getPitchRadians());
        cameraToTarget = target.getBestCameraToTarget();  
      }
    }
  }
  public double getYawDegrees(){
    return m_yaw;
  }
  public double getYawRadians(){
    return Units.degreesToRadians(m_yaw);
  }
  public double getPitchDegrees(){
    return m_pitch;
  }
  public double getPitchRadians(){
    return Units.degreesToRadians(m_pitch);
  }

}
