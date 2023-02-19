// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.FileNotFoundException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Tracking;



public class Robot extends TimedRobot {
  // Define objects and variables
  private Joystick LeftStick;
  private Joystick RightStick;
  private Double RightStickX;
  private Double RightStickY;
  private Double RightStickTwist;
  private Double[] MotorCurrents;
  private String[] AutoNames;
  private String PrevAuto;

  public SwerveDrive SwerveDrive;
  public Tracking Tracking;
  public Autonomous Autonomous;

  public NetworkTableInstance Inst;
  public NetworkTable DriverStation;
  public NetworkTableEntry GyroAng;
  public SendableChooser<String> AutoChooser;

  private CANSparkMax Intake;
  private CANSparkMax Indexer;
  private SparkMaxPIDController IntakePID;

  private Compressor Pump;
  private DoubleSolenoid Intake1;

  @Override
  public void robotInit() {
    Inst = NetworkTableInstance.getDefault();

    AutoChooser = new SendableChooser<String>();
    AutoNames = Filesystem.getDeployDirectory().toPath().resolve("output/paths").toFile().list();
    for (Integer Index = 0; Index <= AutoNames.length - 1; Index++) {
      AutoChooser.addOption(AutoNames[Index], AutoNames[Index]);
    }
    // hehe funne number
    SmartDashboard.putData("AutoChooser", AutoChooser);
    
    // Assign joysticks to the "LeftStick" and "RightStick" objects
    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);

    Intake = new CANSparkMax(9, MotorType.kBrushed);
    Indexer = new CANSparkMax(11, MotorType.kBrushless);

    Pump = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Intake1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    Intake1.set(Value.kReverse);

    // Instantiate an object for each class
    SwerveDrive = new SwerveDrive();
    Tracking = new Tracking(SwerveDrive);
    Autonomous = new Autonomous(SwerveDrive, Tracking);

    // Call SwerveDrive methods, their descriptions are in the SwerveDrive.java file
    SwerveDrive.initMotorControllers(1, 2, 7, 8, 5, 6, 3, 4);
    SwerveDrive.setPID(0.000175, 0.0000007, 0.0000001, 0.0, 8.0, 0.01, 0.01);
    SwerveDrive.initKinematicsAndOdometry();
    PrevAuto = AutoChooser.getSelected();
    Autonomous.AutoFile = AutoChooser.getSelected();
    if (AutoChooser.getSelected() != null) {
    try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
    }
    Pump.enableDigital();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (PrevAuto != AutoChooser.getSelected()) {
      Autonomous.AutoFile = AutoChooser.getSelected();
      try {
        Autonomous.initTrajectory();
      } catch (FileNotFoundException e) {
        System.out.println("AUTO NOT FOUND");
      }
      PrevAuto = AutoChooser.getSelected();
    }
  }
 

  @Override
  public void teleopPeriodic() {
    // Assign stick inputs to variables, to prevent discrepancies
    RightStickX = RightStick.getX();
    RightStickY = RightStick.getY();
    RightStickTwist = RightStick.getRawAxis(3);

    // Create deadzones on the joysticks, to prevent stick drift
    if (Math.abs(RightStickX) < 0.1) {
      RightStickX = 0.0;
    }
    if (Math.abs(RightStickY) < 0.1) {
      RightStickY = 0.0;
    }
    if (Math.abs(RightStickTwist) < 0.2) {
      RightStickTwist = 0.0;
    }

    if (RightStick.getRawButton(6)) {
      Tracking.centerOnCone();
    }
    else {
      // Call swerveDrive() method, to do all the math and outputs for swerve drive
      SwerveDrive.swerveDrive(RightStickX * 2, (RightStickY * -2), (RightStickTwist * 2.5), (1 - ((RightStick.getZ() + 1) / 2)), (1 - ((LeftStick.getZ() + 1) / 2)));
      SwerveDrive.setVariablesAndOptimize();
      SwerveDrive.setSwerveOutputs();
    }

    SmartDashboard.putNumber("Gyro", SwerveDrive.GyroRotation2d.unaryMinus().getDegrees());

    MotorCurrents = new Double[] {SwerveDrive.FrontLeft.Drive.getOutputCurrent(), SwerveDrive.FrontRight.Drive.getOutputCurrent(), SwerveDrive.BackLeft.Drive.getOutputCurrent(), SwerveDrive.BackRight.Drive.getOutputCurrent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", MotorCurrents);
  
    if (RightStick.getRawButton(2) == true) {
      SwerveDrive.Gyro.reset();
    }
    if (RightStick.getRawButton(3)) {
      Intake.set(1);
    }
    else {
      Intake.set(0);
    }
    if (RightStick.getRawButton(5)) {
      Indexer.set(.15);
    }
    else {
      Indexer.set(0);
    }
    if (RightStick.getRawButtonPressed(4)) {
      Intake1.toggle();
    }
  }

  //Autonomous right away
  @Override
  public void autonomousInit(){
    System.out.println("Start Y: " + SwerveDrive.Odometry.getPoseMeters().getY());
  }

  //Autonomous repeat
  @Override
  public void autonomousPeriodic(){ 
    Autonomous.runAutonomous();
  }
}