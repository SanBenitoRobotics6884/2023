package frc.robot.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import static frc.robot.Constants.Drive.*;


public class Trajectory {

    public Command followTrajectoryCommand(PathPlannerTrajectory traj,  boolean isFirstPath, PoseEstimatorSubsystem poseEstimatorSubsystem, DriveSubsystem driveSubsystem) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   poseEstimatorSubsystem.ResetPose2d(traj.getInitialPose());
               }
             }), new PPRamseteCommand(
                traj, poseEstimatorSubsystem::getPose2d, RAMSETE_CONTROLLER, KINEMATICS,
                 driveSubsystem::SetMotorVoltage, driveSubsystem)
             
         );
     }
    
}
