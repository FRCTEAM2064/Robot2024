// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SubsystemCommands.*;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.Constants.Constants.WristConstants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Wrist;
import java.io.File;


import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  final Shooter shooter = new Shooter();
  final Wrist wrist = new Wrist();
  final Intake intake = new Intake();
  final Elevator elevator = new Elevator();
  private final SendableChooser<Command> autoChooser;
  private final PhotonCamera camera;

  private final Joystick leftJoystick = new Joystick(
  OIConstants.kLeftJoystickPort
  );
  private final Joystick rightJoystick = new Joystick(
  OIConstants.kRightJoystickPort
  );


  // XboxController driverController = new XboxController(
  //     OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(
      OIConstants.kOperatorControllerPort);

  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
      new InstantCommand(() -> elevator.setElevatorHeight(9)), 
    new ShootFromIntakeCmd(intake, shooter, wrist, elevator)));

    NamedCommands.registerCommand("IntakeFloorCmd", new IntakeFloorCmd(intake, elevator));
    NamedCommands.registerCommand("HandoffCmd", new HandoffCmd(wrist, shooter, intake, elevator));


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    // () ->
    // MathUtil.applyDeadband(
    // driverController.getLeftY(),
    // OIConstants.kDeadband
    // ),
    // () ->
    // MathUtil.applyDeadband(
    // driverController.getLeftX(),
    // OIConstants.kDeadband
    // ),
    // () -> -driverController.getRawAxis(OIConstants.kXboxRightXAxis),
    // () -> -driverController.getRawAxis(OIConstants.kXboxRightYAxis)
    // );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () ->
        MathUtil.applyDeadband(
          -leftJoystick.getRawAxis(OIConstants.kXboxLeftYAxis),
          OIConstants.kDeadband
        ),
      () ->
        MathUtil.applyDeadband(
          -leftJoystick.getRawAxis(OIConstants.kXboxLeftXAxis),
          OIConstants.kDeadband
        ),
      () -> -rightJoystick.getRawAxis(OIConstants.kXboxRightXAxis)
    );
    // Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    // () ->
    // MathUtil.applyDeadband(
    // driverController.getLeftY(),
    // OIConstants.kDeadband
    // ),
    // () ->
    // MathUtil.applyDeadband(
    // driverController.getLeftX(),
    // OIConstants.kDeadband
    // ),
    // () -> driverController.getRawAxis(OIConstants.kXboxRightXAxis)
    // );

    drivebase.setDefaultCommand(
        // !RobotBase.isSimulation()
        driveFieldOrientedAnglularVelocity
    // driveFieldOrientedDirectAngle
    // : driveFieldOrientedDirectAngleSim
    );
    camera = new PhotonCamera(VisionConstants.kFrontCamName);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // new JoystickButton(operatorController, OIConstants.kXboxYButton)
    //     .whileTrue(new AimAtTagOdometry(drivebase, camera, 6));
    // new JoystickButton(driverController, OIConstants.kXboxYButton)
    // .onTrue(new InstantCommand(shooter::shoot));

    // new JoystickButton(driverController, OIConstants.kXboxLeftBumper)
    // .onTrue(new InstantCommand(() -> wrist.setWristAngle(45)));

    // new JoystickButton(driverController, OIConstants.kXboxRightBumper)
    // .onTrue(
    // new InstantCommand(() -> wrist.setWristAngle(WristConstants.kWristHome))
    // );

    // new JoystickButton(driverController, OIConstants.kXboxStartButton)
    // .onTrue(new InstantCommand(elevator::home));

    // new JoystickButton(driverController, OIConstants.kXboxAButton)
    // .onTrue(new ShootFromIntakeCmd(intake, shooter, wrist));

    // new JoystickButton(driverController, OIConstants.kXboxAButton)
    // .onTrue(new HandoffCmd(wrist, shooter, intake, elevator));

    // new JoystickButton(driverController, OIConstants.kXboxAButton)
    // .onTrue(new FloorToHandoffCmd(shooter, intake, wrist));

    // new JoystickButton(driverController, OIConstants.kXboxBButton)
    // .onTrue(new InstantCommand(()-> elevator.setElevatorHeight(0)));

    // new JoystickButton(driverController, OIConstants.kXboxYButton)
    // .onTrue(new InstantCommand(()-> elevator.setElevatorHeight(0)));

    // new JoystickButton(driverController, OIConstants.kXboxStartButton)
    // .onTrue(new InstantCommand(shooter::feed));

    // new JoystickButton(driverController, OIConstants.kXboxBackButton)
    // .onTrue(new SequentialCommandGroup(new
    // InstantCommand(intake::toggleIntakeMotorIdleMode), new
    // InstantCommand(wrist::toggleWristMotorIdleMode)));

    // new JoystickButton(driverController, OIConstants.kXboxLeftStickButton)
    // .onTrue(new InstantCommand(drivebase::zeroGyro));

    // new JoystickButton(driverController, OIConstants.kXboxXButton)
    // .whileTrue(new IntakeFloorCmd(intake, elevator));

    // new JoystickButton(driverController, OIConstants.kXboxBButton)
    // .onTrue(new InstantCommand(() -> elevator.setElevatorHeight(6)));

    // -----------------COMPETITION--------------------------

    // DRIVER MAPPING
    
      new JoystickButton(leftJoystick, OIConstants.kJoystickTriggerButton)
      .whileTrue(new SequentialCommandGroup(
        new InstantCommand(() -> intake.setPivotSpeed(.2)).withTimeout(.4)
      .andThen(new InstantCommand(() -> intake.setPivotSpeed(0))), 
      new IntakeFloorCmd(intake, elevator)));
      
      new JoystickButton(leftJoystick, 8)
      .onTrue(new InstantCommand(drivebase::zeroGyro));
      
      new JoystickButton(rightJoystick, OIConstants.kJoystickTriggerButton)
      .whileTrue(new SequentialCommandGroup(
        new InstantCommand(() -> intake.setIntakeAngle(210.0)),
        new WaitCommand(1.5),
        new InstantCommand(intake::outtake), new InstantCommand(() -> intake.setIntakeAngle(IntakeConstants.kIntakeHome))));

      new JoystickButton(leftJoystick, 10)
      .onTrue(new SequentialCommandGroup(new
      InstantCommand(intake::toggleIntakeMotorIdleMode), new
      InstantCommand(wrist::toggleWristMotorIdleMode)));

      
      
      // OPERATOR MAPPING
      new JoystickButton(operatorController, OIConstants.kXboxStartButton)
      .onTrue(new InstantCommand(elevator::home));


      new JoystickButton(operatorController, OIConstants.kXboxLeftBumper)
      .whileTrue(new ElevatorMoveCmd(elevator, 0.5));
      
      new JoystickButton(operatorController, OIConstants.kXboxRightBumper)
      .whileTrue(new ElevatorMoveCmd(elevator, -0.5));
      
      // new JoystickButton(operatorController, OIConstants.kXboxBButton)
      // .onTrue(new SequentialCommandGroup(new InstantCommand(() ->
      // wrist.setWristAngle(35)), new InstantCommand(shooter::shoot)));


      new JoystickButton(operatorController, OIConstants.kXboxBButton).
      onTrue(new ShootFromIntakeCmd(intake, shooter, wrist, elevator));



      
      new JoystickButton(operatorController, OIConstants.kXboxBackButton)
      .whileTrue(new InstantCommand(intake::outtake));
      
      //Climb
      new JoystickButton(operatorController, OIConstants.kXboxXButton)
      .onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> elevator.setElevatorHeight(5)),
      new InstantCommand(() -> wrist.setWristAngle(100))));

      
      new JoystickButton(operatorController, OIConstants.kXboxAButton)
      .onTrue(new HandoffCmd(wrist, shooter, intake, elevator));      
      
      
      // new JoystickButton(operatorController, OIConstants.kXboxStartButton)
      // .onTrue(new InstantCommand(shooter::overrideHasGamePiece));
      
      // new JoystickButton(operatorController, OIConstants.kXboxYButton)
      // .onTrue(new ScoreAmpCmd(elevator, wrist, shooter));
      
      new JoystickButton(operatorController, OIConstants.kXboxYButton)
      .onTrue(new SequentialCommandGroup(new InstantCommand(shooter::setStateAmp), 
      new WaitCommand(2), 
      new InstantCommand(() -> wrist.setWristAngle(WristConstants.kWristHome))));
      
      
      
      //SHUFFLEBOARD COMMANDS
      SmartDashboard.putData("Clean Wheels", new
      InstantCommand(shooter::cleanFeed));
  }

  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("Straight");
    // return AutoBuilder.followPath(path);
    return autoChooser.getSelected();
  }
}
