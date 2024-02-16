// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SubsystemCommands.IntakeFloorCmd;
import frc.robot.Commands.SubsystemCommands.PostionCommands.WristCmd;
import frc.robot.Constants.Constants.OIConstants;
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
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
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
      () -> driverController.getRightX(),
      () -> driverController.getRightY()
    );

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
      () -> driverController.getRawAxis(2)
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
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
      () -> driverController.getRawAxis(2)
    );

    drivebase.setDefaultCommand(
      !RobotBase.isSimulation()
        ? driveFieldOrientedAnglularVelocity
        : driveFieldOrientedDirectAngleSim
    );

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    new JoystickButton(driverController, OIConstants.kXboxYButton)
      .onTrue((new InstantCommand(drivebase::zeroGyro)));

    new JoystickButton(driverController, OIConstants.kXboxXButton)
      .onTrue(new InstantCommand(shooter::shoot));

    new JoystickButton(driverController, OIConstants.kXboxLeftBumper)
      .onTrue(new WristCmd(wrist, 10));

    new JoystickButton(driverController, OIConstants.kXboxRightBumper)
      .whileTrue(
        new RepeatCommand(new InstantCommand(drivebase::lock, drivebase))
      );

    new JoystickButton(driverController, OIConstants.kXboxLeftTriggerAxis)
    .whileTrue(new IntakeFloorCmd(intake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
