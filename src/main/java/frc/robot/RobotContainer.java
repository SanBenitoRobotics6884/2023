// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.RobotConstants.Claw.*;
import static frc.robot.constants.RobotConstants.Drive.*;
import static frc.robot.constants.RobotConstants.FiducialTracking.*;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.astar.Edge;
import frc.robot.astar.Node;
import frc.robot.astar.Obstacle;
import frc.robot.astar.VisGraph;
import frc.robot.commands.AStar;
import frc.robot.commands.ClawCmmd;
import frc.robot.commands.DriveCmmd;
import frc.robot.commands.PivotCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final ExtendSubsystem m_extendSubsystem = new ExtendSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_gyro);
  private final PoseEstimatorSubsystem poseEstimatorSubsystem = new PoseEstimatorSubsystem(CAMERA_ONE, m_driveSubsystem);
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
 
  private final Joystick m_joystick = new Joystick(0);
  private final CommandXboxController controller = new CommandXboxController(1);
  private final Command m_extendCommand = new PivotCommand(m_pivotSubsystem,
  () -> m_joystick.getY());
  private final Command m_pivotCommand = new PivotCommand(m_pivotSubsystem,
      () -> m_joystick.getY());
  private final ClawCmmd m_clawCommand = new ClawCmmd(
    m_clawSubsystem,
    () -> m_joystick.getTrigger(),
    () -> m_joystick.getRawButton(3),
    () -> m_joystick.getRawButton(4));
  private final DriveCmmd m_normalDriveCommand = new DriveCmmd(
      m_driveSubsystem,
      () -> controller.getLeftY(),
      () -> -controller.getRightX(),
      false);
  private final DriveCmmd m_snailDriveCommand = new DriveCmmd(
      m_driveSubsystem,
      () -> controller.getLeftY(),
      () -> -controller.getRightX(),
      true);

  // final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  private final List<Obstacle> obstacles = FieldConstants.obstacles;
  private final VisGraph AStarMap = new VisGraph();
 
  HashMap<String, Command> eventMap;
  RunCommand autoBalance = new RunCommand(
    m_driveSubsystem::chargeStationAlign, m_driveSubsystem);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pivotSubsystem.setDefaultCommand(m_pivotCommand);
    m_extendSubsystem.setDefaultCommand(m_extendCommand);
    eventMap = new HashMap<>();
    eventMap.put("autobalance", autoBalance);
    eventMap.put("highScore", m_pivotSubsystem.getPlaceCommand().andThen(new WaitCommand(1)).andThen(m_extendSubsystem.getExtendCommand())  );
   
    m_gyro.calibrate();
    m_clawSubsystem.setDefaultCommand(m_clawCommand);
    m_driveSubsystem.setDefaultCommand(m_normalDriveCommand);
   

   
    
    AStarMap.addNode(new Node(2.48 - 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 4.42 + 0.1));
    AStarMap.addNode(new Node(5.36 + 0.1, 1.07 - 0.1));
    AStarMap.addNode(new Node(2.48 - 0.1, 1.07 - 0.1));
    AStarMap.addNode(new Node(3.84 + 0.1, 4.80 - 0.1)); // Divider
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node startNode = AStarMap.getNode(i);
      for (int j = i + 1; j < AStarMap.getNodeSize(); j++) {
        AStarMap.addEdge(new Edge(startNode, AStarMap.getNode(j)), obstacles);
      }
    }
    configureButtonBindings();
  }

  
  private void configureButtonBindings() {
    // Chassis triggers
    controller.leftTrigger()
        .whileTrue(m_snailDriveCommand);

    controller.x().whileTrue(new AStar(
        m_driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), 
        Rotation2d.fromDegrees(180)), obstacles, AStarMap));

    controller.y().whileTrue(new AStar(
        m_driveSubsystem, poseEstimatorSubsystem,
        new PathConstraints(2, 1.5), new Node(new Translation2d(2.0146, 2.75), 
        Rotation2d.fromDegrees(180)), obstacles, AStarMap));

    controller.a().onTrue(new InstantCommand(m_driveSubsystem::resetEncoders));

    controller.b().whileTrue(autoBalance);
    
    // Claw triggers
    new JoystickButton(m_joystick, 2)
        .onTrue(new InstantCommand(m_clawSubsystem::colorCheck)); // To close the claw (with color sensor) 
    new JoystickButton(m_joystick, 1).negate()
        .and(new JoystickButton(m_joystick, 3))
        .and(new JoystickButton(m_joystick, 4))
        .onTrue(new InstantCommand(() -> m_clawSubsystem.setRotations(OPEN_SETPOINT))); 

    // Extend setpoint triggers
    new JoystickButton(m_joystick, 11)
         .onTrue(m_extendSubsystem.getRetractCommand()); 

    new JoystickButton(m_joystick, 9)
        .onTrue(m_extendSubsystem.getMidCommand());

    new JoystickButton(m_joystick, 7)
        .onTrue(m_extendSubsystem.getExtendCommand());

    // Pivot setpoint triggers
    new JoystickButton(m_joystick, 12)
        .onTrue(m_pivotSubsystem.getDownCommand());
    
    new JoystickButton(m_joystick, 10)
        .onTrue(m_pivotSubsystem.getPickUpCommand());
    
    new JoystickButton(m_joystick, 8)
        .onTrue(m_pivotSubsystem.getPlaceCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        
    return makeAutoBuilderCommand("A", CONSTRAINTS);
  }
 
 
  private CommandBase makeAutoBuilderCommand(String pathName, PathConstraints constraints) {
   
    var path = PathPlanner.loadPath(pathName, constraints);
    
    poseEstimatorSubsystem.AddTrajectory(path);
   
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(poseEstimatorSubsystem::getPose2d, poseEstimatorSubsystem::ResetPose2d,
         RAMSETE_CONTROLLER, KINEMATICS,  m_driveSubsystem::tankDrive, eventMap, m_driveSubsystem );
         /*new RamseteAutoBuilder(poseEstimatorSubsystem::getPose2d, poseEstimatorSubsystem::ResetPose2d, RAMSETE_CONTROLLER,
          KINEMATICS, FEED_FOWARD, m_driveSubsystem::getWheelSpeeds, new PIDConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD), m_driveSubsystem::tankDrive,
           m_hashMap, m_driveSubsystem);*/
    return autoBuilder.fullAuto(path);
}
}
