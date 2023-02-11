// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.ConstantsFolder.RobotConstants.FiducialTracking.*;

import java.util.Optional;


public class VisionSubsystem extends SubsystemBase {
  double m_yaw;
  double m_pitch;
  double m_distance;
  int m_fiducialID;
  PhotonCamera m_camera;
  DriveSubsystem m_driveSubsystem;
  PhotonTrackedTarget m_target;
  double m_targetHeight;
  Pose3d m_tagPose3d;
  Transform3d cameraToTarget;
  
  /** Creates a new VisionSubsystem. */
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
    if(result.hasTargets()) {
      m_target = result.getBestTarget();
      if(m_target.getPoseAmbiguity() <= .2) {
        m_fiducialID = m_target.getFiducialId();
        m_yaw = m_target.getYaw();
        m_pitch = m_target.getPitch();
        m_tagPose3d = TAG_POSES.getTagPose(m_fiducialID).get();
        m_distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_ONE_HEIGHT, m_tagPose3d.getZ(), CAMERA_ONE_PITCH_RADIANS, GetPitchRadians());
        cameraToTarget = m_target.getBestCameraToTarget();     
      }
    }
  }

  public double GetYawDegrees() {
    return m_yaw;
  }

  public double GetYawRadians() {
    return m_yaw * Math.PI /180;
  }

  public double GetPitchDegrees() {
    return m_pitch;
  }

  public double GetPitchRadians() {
    return m_pitch * Math.PI /180;
  }

  public Pose3d getCameraPose() {
    Transform3d cameraToTarget = m_target.getBestCameraToTarget();
    return m_tagPose3d.transformBy(cameraToTarget.inverse());
  }

  public Pose3d getRobotPose() {
    return getCameraPose().transformBy(CAMERA_TO_ROBOT);
  }

  public Pose3d getTurretPose() {
    return getCameraPose().transformBy(CAMERA_TO_TURRET);
  }

  /** Calculate the number of rotations required to look at the cube node */
  public double calculateCube() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = m_tagPose3d.transformBy(TAG_TO_CUBE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }

  /** Calculate the number of rotations required to look at left node */
  public double calculateLeftNode() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = m_tagPose3d.transformBy(TAG_TO_LEFT_CONE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }

  /** Calculate the number of rotations required to look at the right node */
  public double calculateRightNode() {
    Pose3d turretPose = getTurretPose();
    Pose3d nodePose = m_tagPose3d.transformBy(TAG_TO_RIGHT_CONE);
    return Units.radiansToRotations(
      turretPose.getRotation().getZ()
      - Math.atan2(
        nodePose.getY() - turretPose.getY(),
        nodePose.getX() - turretPose.getX()));
  }
}
