// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.function.Supplier;

import org.ejml.ops.ReadCsv;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.io.*;

public class Autonomous extends SubsystemBase {
  public Tracking Tracking;
  private SwerveDrive Swerve;
  public String AutoFile;
  private Path TrajectoryPath;
  private File TrajectoryFile;
  private TrajectoryConfig TrajectoryConfig;
  private Trajectory PlaceFirstObjectTrajectory;
  private Trajectory GetSecondObjectTrajectory;
  private Trajectory PlaceSecondObjectTrajectory;
  private Trajectory ToChargerTrajectory;
  private SwerveDriveKinematicsConstraint SwerveDriveMaxSpeed;
  private Constraints PIDConstraints;
  private Scanner AutoReader;
  private List<String> Lines;
  private List<String> CurrentLine;
  private List<Double> XPositions;
  private List<Double> YPositions;
  private List<Double> Angles;
  private Boolean HasChargerAuto = false;
  private SwerveControllerCommand PlaceFirstObject;
  private SwerveControllerCommand GetSecondObject;
  private SwerveControllerCommand PlaceSecondObject;
  private SwerveControllerCommand ToCharger;
  private Integer AutoStage;
  private Boolean IsScheduled = false;

  public Autonomous(SwerveDrive SwerveDrive, Tracking Track) {
    Swerve = SwerveDrive;
    Tracking = Track;
    Lines = new ArrayList<String>();
    CurrentLine = new ArrayList<String>();
    XPositions = new ArrayList<Double>();
    YPositions = new ArrayList<Double>();
    Angles = new ArrayList<Double>();
    AutoStage = 0;
  }

  public void initTrajectory() throws FileNotFoundException {
    TrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + AutoFile);
    System.out.println(TrajectoryPath.toString());
    TrajectoryFile = TrajectoryPath.toFile();
    AutoReader = new Scanner(TrajectoryFile);
    SwerveDriveMaxSpeed = new SwerveDriveKinematicsConstraint(Swerve.Kinematics, 1);
    PIDConstraints = new Constraints(1, 2);
    TrajectoryConfig = new TrajectoryConfig(1, 1).setKinematics(Swerve.Kinematics).addConstraint(SwerveDriveMaxSpeed);
    Lines.clear();
    CurrentLine.clear();
    XPositions.clear();
    YPositions.clear();
    Angles.clear();
    AutoStage = 0;
    Swerve.Gyro.reset();

    if (AutoReader.hasNextLine()) {
      AutoReader.nextLine();
    }
    while (AutoReader.hasNextLine()) {
      Lines.add(AutoReader.nextLine());
    }
    for (Integer Index = 0; Index <= Lines.size() - 1; Index++) {
      CurrentLine.addAll(Arrays.asList(Lines.get(Index).split(",")));
      XPositions.add(Double.parseDouble(CurrentLine.get(0)));
      YPositions.add(Double.parseDouble(CurrentLine.get(1)));
      System.out.println(CurrentLine.get(1));
      if (TrajectoryPath.toString().contains("Red")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))) + Math.PI);
      }
      if (TrajectoryPath.toString().contains("Blue")) {
        Angles.add(Math.atan2(Double.parseDouble(CurrentLine.get(3)), Double.parseDouble(CurrentLine.get(2))));
      }
      if (Index == 6) {
        HasChargerAuto = true;
      }
      CurrentLine.clear();
    }
    AutoReader.close();
    Swerve.Odometry.resetPosition(new Rotation2d(Angles.get(0)), new SwerveModulePosition[] {Swerve.FrontRight.getPosition(), Swerve.FrontLeft.getPosition(), Swerve.BackLeft.getPosition(), Swerve.BackRight.getPosition()}, new Pose2d(new Translation2d(XPositions.get(0), YPositions.get(0)), new Rotation2d(0)));

    PlaceFirstObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(0), YPositions.get(0)), new Rotation2d(Angles.get(0))), List.of(), new Pose2d(new Translation2d(XPositions.get(1), YPositions.get(1)), new Rotation2d(Angles.get(1))), TrajectoryConfig);
    //GetSecondObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(1), YPositions.get(1)), new Rotation2d(Angles.get(1))), List.of(new Translation2d(XPositions.get(2), YPositions.get(2))), new Pose2d(new Translation2d(XPositions.get(3), YPositions.get(3)), new Rotation2d(Angles.get(3))), TrajectoryConfig);
    //PlaceSecondObjectTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(3), YPositions.get(3)), new Rotation2d(Angles.get(3))), List.of(new Translation2d(XPositions.get(4), YPositions.get(4))), new Pose2d(new Translation2d(XPositions.get(5), YPositions.get(5)), new Rotation2d(Angles.get(5))), TrajectoryConfig);
    //ToChargerTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(XPositions.get(5), YPositions.get(5)), new Rotation2d(Angles.get(5))), List.of(), new Pose2d(new Translation2d(XPositions.get(6), YPositions.get(6)), new Rotation2d(Angles.get(6))), TrajectoryConfig);

    PlaceFirstObject = new SwerveControllerCommand(PlaceFirstObjectTrajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve);
    //GetSecondObject = new SwerveControllerCommand(GetSecondObjectTrajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve);
    //PlaceSecondObject = new SwerveControllerCommand(PlaceSecondObjectTrajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve);
    //ToCharger = new SwerveControllerCommand(ToChargerTrajectory, Swerve::getPose, Swerve.Kinematics, new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, PIDConstraints)), Swerve::setModuleStates, Swerve);
  }

  public void runAutonomous() {
    if (AutoStage == 0) {
      if (!IsScheduled) {
        PlaceFirstObject.andThen(() -> Swerve.stop()).schedule();
        IsScheduled = true;
      }
      if (PlaceFirstObject.isFinished()) {
        AutoStage = 1;
        IsScheduled = false;
      }
    }
    /**
    if (AutoStage == 1) {
      //Extend arm
      //Line up with Limelight
      //Drop object
      //Retract arm
      AutoStage = 2;
    }
    if (AutoStage == 2) {
      if (!IsScheduled) {
        GetSecondObject.andThen(() -> Swerve.stop()).schedule();
        IsScheduled = true;
      }
      if (GetSecondObject.isFinished()) {
        AutoStage = 3;
        IsScheduled = false;
      }
    }
    if (AutoStage == 3) {
      //Extend intake
      //Start motors
      //Pick up with Limelight
      //Stop motors
      //Retract intake
      AutoStage = 4;
    }
    if (AutoStage == 4) {
      if (!IsScheduled) {
        PlaceSecondObject.andThen(() -> Swerve.stop()).schedule();
        IsScheduled = true;
      }
      if (PlaceSecondObject.isFinished()) {
        AutoStage = 5;
        IsScheduled = false;
      }
    }
    if (AutoStage == 5) {
      //Extend arm
      //Line up with Limelight
      //Drop object
      //Retract arm
      AutoStage = 6;
    }
    if (AutoStage == 6 & HasChargerAuto) {
      if (!IsScheduled) {
        ToCharger.andThen(() -> Swerve.stop()).schedule();
        IsScheduled = true;
      }
      if (ToCharger.isFinished()) {
        AutoStage = 7;
        IsScheduled = false;
      }
    }
    if (AutoStage == 7) {
      //Auto balance on charger
    }
    */
  }
}