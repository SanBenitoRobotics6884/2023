// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ConstantsFolder;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.equation.Variable;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants {

    public static final class Drive {
        public static final int FR_ID = 32;
        public static final int FL_ID = 33;
        public static final int BR_ID = 30;
        public static final int BL_ID = 31;

        public static final double NORMAL_FOWARD_FF = 0.0;
        public static final double NORMAL_TURN_FF = 0.7;
        public static final double TURBO_FOWARD_FF = 0.0;
        public static final double TURBO_TURN_FF = 0.1;

        public static final int ENCODER_REVOLUTION= 8192;
        public static final double GEAR_RATIO = 1/10.71;
        public static final double WHEEL_RADIUS = 3.0;
      
        public static final double POSITION_CONVERSION =WHEEL_RADIUS*Math.PI *2 *GEAR_RATIO*10;
        public static final double VELOCITY_CONVERSION =WHEEL_RADIUS*Math.PI *2 *GEAR_RATIO*10/60;

        public static final double TURN_TO_TARGET_KP = 1.2;
        public static final double TURN_TO_TARGET_KI = 0.0;
        public static final double TURN_TO_TARGET_KD = 0.0;
        public static final PIDController TURN_TO_TARGET_CONTROLLER = 
        new PIDController(TURN_TO_TARGET_KP, TURN_TO_TARGET_KI, TURN_TO_TARGET_KD);
        public static final double TURN_TO_TARGET_FF = 0.0;
       
        public static final int PIGEON_ID = 0;
        public static final double MOUNT_YAW = 0.0 ;
        public static final double MOUNT_PITCH = 0.0 ;
        public static final double MOUNT_ROLL = 0.0 ;
        public static final double GRAVITY_VECTOR[] = new double[3];

        public static final double DRIVE_KP = 0.0;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;  
        public static final PIDController RIGHT_DRIVE_CONTROLLER = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);  
        public static final PIDController LEFT_DRIVE_CONTROLLER = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);  

        public static final double AUTO_BALANCE_P = 0.0;
        public static final double AUTO_BALANCE_I = 0.0;
        public static final double AUTO_BALANCE_D = 0.0;
        public static final PIDController AUTO_BALANCE_CONTROLLER = new PIDController(AUTO_BALANCE_P, AUTO_BALANCE_I, AUTO_BALANCE_D);

        /*public static final double TELE_FOWARD_P = 0.0;
        public static final double TELE_FOWARD_I = 0.0;
        public static final double TELE_FOWARD_D = 0.0;
        public static final PIDController TELE_FOWARD_CONTROLLER = new PIDController(TELE_FOWARD_P, TELE_FOWARD_I, TELE_FOWARD_D);
*/
        public static final double TELE_ROTATION_P = 0.7;
        public static final double TELE_ROTATION_I = 0.0;
        public static final double TELE_ROTATION_D = 0.0;
        public static final PIDController TELE_ROTATION_CONTROLLER = new PIDController(TELE_ROTATION_P, TELE_ROTATION_I, TELE_ROTATION_D);
        
        public static final double KA = 0.0;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double RAMSETE_ZETA = 0.0;
        public static final double RAMSETE_B = 0.0;

         
        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController(RAMSETE_B, RAMSETE_ZETA);
     
        public static final SimpleMotorFeedforward FEED_FOWARD = new SimpleMotorFeedforward(KS, KV);
        //not done
        public static final double TRACK_WIDTH = Units.inchesToMeters(2.0);
        public static final DifferentialDriveKinematics KINEMATICS = 
        new DifferentialDriveKinematics(TRACK_WIDTH);
       
        public static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(FEED_FOWARD, KINEMATICS, 11.0);
        

        public static final double MAX_VELOCTIY = 0.0; 
        public static final double MAX_ACCELERATION = 0.0; 
        public static final PathConstraints CONSTRAINTS = new PathConstraints(MAX_VELOCTIY, MAX_ACCELERATION);

        public static final TrajectoryConfig TRAJECTORY_CONFIG =
         new TrajectoryConfig(MAX_VELOCTIY, MAX_ACCELERATION).setKinematics(KINEMATICS).addConstraint(AUTO_VOLTAGE_CONSTRAINT); 

    }
    public static final class FiducialTracking{

        public static final String CAMERA_ONE_NAME = "";
        public static final PhotonCamera CAMERA_ONE = new PhotonCamera(CAMERA_ONE_NAME);
        public static final double CAMERA_ONE_HEIGHT = Units.inchesToMeters(0.0);
        public static final double CAMERA_ONE_PITCH_RADIANS = Units.degreesToRadians(0.0);

        public static final Pose3d TAG_ONE_POSE = new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22), 
            new Rotation3d(0, 0,Units.degreesToRadians(180)) );

        public static final Pose3d TAG_TWO_POSE = new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0, 0,Units.degreesToRadians(180)) );

        public static final Pose3d TAG_THREE_POSE = new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), 
            Units.inchesToMeters(18.22),
            new Rotation3d(0, 0,Units.degreesToRadians(180)) );

        public static final Pose3d TAG_FOUR_POSE = new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0, 0,Units.degreesToRadians(180)) );

        public static final Pose3d TAG_FIVE_POSE = new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0, 0,Units.degreesToRadians(0)) );
             
        public static final Pose3d TAG_SIX_POSE = new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0, 0,Units.degreesToRadians(0)) );
             
        public static final Pose3d TAG_SEVEN_POSE = new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0, 0,Units.degreesToRadians(0)) );

        public static final Pose3d TAG_EIGHT_POSE = new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0, 0,Units.degreesToRadians(0)) );

        public static final AprilTag TAG_ONE = new AprilTag(1, TAG_ONE_POSE);
        public static final AprilTag TAG_TWO = new AprilTag(2, TAG_TWO_POSE);
        public static final AprilTag TAG_THREE = new AprilTag(3, TAG_THREE_POSE);
        public static final AprilTag TAG_FOUR = new AprilTag(4, TAG_FOUR_POSE);
        public static final AprilTag TAG_FIVE = new AprilTag(5, TAG_FIVE_POSE);
        public static final AprilTag TAG_SIX = new AprilTag(6, TAG_SIX_POSE);
        public static final AprilTag TAG_SEVEN = new AprilTag(7, TAG_SEVEN_POSE);
        public static final AprilTag TAG_EIGHT = new AprilTag(8, TAG_EIGHT_POSE);

        public static final List<AprilTag> TAG_LIST = (List<AprilTag>) Collections.unmodifiableList(
        List.of(TAG_ONE, TAG_TWO, TAG_THREE, TAG_FOUR, TAG_FIVE, TAG_SIX, TAG_SEVEN, TAG_EIGHT));

        public static final double FIELD_LENGTH = Units.inchesToMeters(319);
        public static final double FIELD_WIDTH = Units.inchesToMeters(649);
        public static final AprilTagFieldLayout TAG_POSES = new AprilTagFieldLayout(
            TAG_LIST, FIELD_LENGTH, FIELD_WIDTH);

        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
        public static final Transform3d BOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
     
    }
  
  
   
}
