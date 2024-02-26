// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SubsystemCommands.*;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Wrist;
import java.io.File;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve/neo")
  );
  final Shooter shooter = new Shooter();
  final Wrist wrist = new Wrist();
  final Intake intake = new Intake();
  final Elevator elevator = new Elevator();
  private final SendableChooser<Command> autoChooser;

  // private final Joystick driverController = new Joystick(
  //   OIConstants.kDriverControllerPort
  // );

  XboxController driverController = new XboxController(
    OIConstants.kDriverControllerPort
  );
  XboxController operatorController = new XboxController(
    OIConstants.kOperatorControllerPort
  );

  public RobotContainer() {
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //   () ->
    //     MathUtil.applyDeadband(
    //       driverController.getLeftY(),
    //       OIConstants.kDeadband
    //     ),
    //   () ->
    //     MathUtil.applyDeadband(
    //       driverController.getLeftX(),
    //       OIConstants.kDeadband
    //     ),
    //   () -> driverController.getRawAxis(OIConstants.kXboxRightXAxis),
    //   () -> driverController.getRawAxis(OIConstants.kXboxRightYAxis)
    // );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () ->
        MathUtil.applyDeadband(
          driverController.getLeftY(),
          OIConstants.kDeadband
        ),
      () ->
        MathUtil.applyDeadband(
          driverController.getLeftX(),
          OIConstants.kDeadband
        ),
      () -> driverController.getRawAxis(OIConstants.kXboxRightXAxis)
    );

    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //   () ->
    //     MathUtil.applyDeadband(
    //       driverController.getLeftY(),
    //       OIConstants.kDeadband
    //     ),
    //   () ->
    //     MathUtil.applyDeadband(
    //       driverController.getLeftX(),
    //       OIConstants.kDeadband
    //     ),
    //   () -> driverController.getRawAxis(OIConstants.kXboxRightXAxis)
    // );

    drivebase.setDefaultCommand(
      //!RobotBase.isSimulation()
      driveFieldOrientedAnglularVelocity
      //: driveFieldOrientedDirectAngleSim
    );

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    new JoystickButton(driverController, OIConstants.kXboxYButton)
      .onTrue(new InstantCommand(shooter::shoot));

    new JoystickButton(driverController, OIConstants.kXboxLeftBumper)
      .onTrue(new InstantCommand(() -> wrist.setWristAngle(45)));

    new JoystickButton(driverController, OIConstants.kXboxRightBumper)
      .onTrue(
        new InstantCommand(() -> wrist.setWristAngle(WristConstants.kWristHome))
      );

    // new JoystickButton(driverController, OIConstants.kXboxAButton)
    //   .onTrue(new ShootFromIntakeCmd(intake, shooter, wrist));

    new JoystickButton(driverController, OIConstants.kXboxAButton)
      .onTrue(new HandoffCmd(wrist, shooter, intake, elevator));

    //     new JoystickButton(driverController, OIConstants.kXboxAButton)
    // .onTrue(new FloorToHandoffCmd(shooter, intake, wrist));

    // new JoystickButton(driverController, OIConstants.kXboxBButton)
    //   .onTrue(new InstantCommand(()-> elevator.setElevatorHeight(0)));

    // new JoystickButton(driverController, OIConstants.kXboxYButton)
    //   .onTrue(new InstantCommand(()-> elevator.setElevatorHeight(0)));

    new JoystickButton(driverController, OIConstants.kXboxStartButton)
      .onTrue(new InstantCommand(shooter::feed));

    new JoystickButton(driverController, OIConstants.kXboxLeftStickButton)
      .onTrue(new InstantCommand(drivebase::zeroGyro));

    new JoystickButton(driverController, OIConstants.kXboxXButton)
      .whileTrue(new IntakeFloorCmd(intake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
