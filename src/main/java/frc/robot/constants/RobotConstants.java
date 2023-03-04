// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

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
        public static final int FR_ID = 1;
        public static final int FL_ID = 2;
        public static final int BR_ID = 3;
        public static final int BL_ID = 4;

        public static final double NORMAL_FOWARD_FF = 0.7;
        public static final double NORMAL_TURN_FF = 0.7;
        public static final double TURBO_FOWARD_FF = 0.9;
        public static final double TURBO_TURN_FF = 0.1;

        public static final int ENCODER_REVOLUTION= 8192;
        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_RADIUS = 3.0;
      
        public static final double POSITION_CONVERSION = 
        (Units.inchesToMeters(1 / (GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS)) * 10));
        public static final double VELOCITY_CONVERSION = (POSITION_CONVERSION/60);

       

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

        public static final double DRIVE_KP = 0.088307;
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
        
        public static final double KA = 0.11778;
        public static final double KS = 0.056226;
        public static final double KV = 2.8715;
        public static final double RAMSETE_ZETA = 0.7;
        public static final double RAMSETE_B = 2.0;

         
        public static final RamseteController RAMSETE_CONTROLLER = new RamseteController(RAMSETE_B, RAMSETE_ZETA);
     
        public static final SimpleMotorFeedforward FEED_FOWARD = new SimpleMotorFeedforward(KS, KV);
        //23 inches when using center of the wheels, 21 inches when going from end to end
        public static final double TRACK_WIDTH = Units.inchesToMeters(23);
        public static final DifferentialDriveKinematics KINEMATICS = 
        new DifferentialDriveKinematics(TRACK_WIDTH);
       
        public static final DifferentialDriveVoltageConstraint AUTO_VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(FEED_FOWARD, KINEMATICS, 11.0);
        

        public static final double MAX_VELOCTIY = 1.0; 
        public static final double MAX_ACCELERATION = 1.0; 
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
  
    public static class Claw {
        public static final int MOTOR_ID = 8;

        public static final double P = 0.25;
        public static final double I = 0;
        public static final double D = 0.25; // Might or might not use
        public static final double MAX_VOLTAGE = 0.25; // Might need more testing
        public static final double CONE_MIN_HUE = 70;
        public static final double CONE_MAX_HUE = 95;
        public static final double CUBE_MIN_HUE = 140;
        public static final double CUBE_MAX_HUE = 230;

        public static final double GEAR_RATIO = 50;
        public static final double CONE_SETPOINT = -8.1; // needs adjustment
        public static final double CUBE_SETPOINT = -4; // needs adjustment
        public static final double OPEN_SETPOINT = 0; 
        public static final double CLOSED_SETPOINT = -8.9; // needs adjustment
        public static final double OPEN_RATE = 0.05; // Might need adjustment
        public static final double CLOSE_RATE = -0.05;
    }
  
    public static final class Arm {

        public static final class Pivot {
            public static final double P = 0.4;
            public static final double I = 0;
            public static final double D = 0;
            public static final double CANCODER_COEFFICIENT = 1.0 / 4096; // Rotations
            public static final double CANCODER_OFFSET_DEGREES = 0; // Needs significant testing if it is even usable

            public static final double GEAR_RATIO = 9;
            public static final double MAX_VOLTAGE = 0.15; // For normal pid. DOUBLE CHECK NEEDED
            public static final double MAX_VELOCITY = 0; // For profiled pid (needs testing)
            public static final double MAX_ACCELERATION = 0;

            public static final double HYBRID_SETPOINT = GEAR_RATIO * 0.2; // NEED SETPOINTS
            public static final double MID_SETPOINT = GEAR_RATIO * 0.3;
            public static final double HIGH_SETPOINT = GEAR_RATIO * 0.4;
            
            public static final double Y_SCALE = 0.025;

            // Rotations of the shaft that the CANCoder is attached to
            public static final double BACK_HARD_LIMIT = 0; // Limits where the setpoint can go on the joystick
            public static final double FRONT_HARD_LIMIT = GEAR_RATIO * 0.45; // NEED ANGLE

            public static final int PIVOT_CANCODER_ID = 0;
            public static final int MASTER_MOTOR_ID = 5; 
            public static final int SLAVE_MOTOR_ID = 6;

            public static final int SWITCH_PORT = 1;

           
        
        }

        public static final class Extend {
            public static final double P = 0.2;
            public static final double I = 0;
            public static final double D = 0;
            public static final double SETPOINT_TOLERANCE = 4;
            public static final double START_EXTENDING = 1;
            

            public static final double HYBRID_SETPOINT = 0;
            public static final double MID_SETPOINT = 25;
            public static final double HIGH_SETPOINT = 35;

            public static final double Y_SCALE = 0.06;

            public static final float EXTEND_REVERSE_SOFT_LIMIT = 1;
            public static final float EXTEND_FORWARD_SOFT_LIMIT = 1;
            public static final double BACK_HARD_LIMIT = -10;
            public static final double FRONT_HARD_LIMIT = 80;
            public static final double FULLY_RETRACTED = 0;
            public static final double FULLY_EXTENDED = 0;

            public static final int EXTEND_MOTOR_ID = 7; 

            public static final double GEAR_RATIO = 7;
            // All voltage values are percent output
            public static final double BACK_VOLTAGE = 0.01; // Should be positive (has negative sign in code)
            public static final double BACK_TIME = 0.1; // Should be between 0 and SERVO_DELAY
            public static final double MAX_VOLTAGE_EXTEND = 0.2;
            public static final double MAX_VOLTAGE_RETRACT = 0.2;
            public static final double MAX_VELOCITY = 0; // profiled pid 
            public static final double MAX_ACCELERATION = 0;

            public static final int SERVO_PORT = 9;
            public static final double SERVO_DELAY = 1;
            public static final double RATCHET_ENGAGED = 115. / 180;
            public static final double RATCHET_DISENGAGED = 93. / 180;
            public static final double RATCHET_DELAY = 1;
        }
    }  
}
