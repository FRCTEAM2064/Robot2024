package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkFlex leaderShooterMotor;
  private CANSparkFlex followerShooterMotor;
  private CANSparkMax feederMotor;

  private RelativeEncoder leaderShooterEncoder;
  public Timer feedTimer = new Timer();

  private ShooterState state = ShooterState.STOP;

  private boolean gotIntoFeed = false;
  private Integer counter = 0;
  private boolean shooting = false;

  public Shooter() {
    leaderShooterMotor =
      new CANSparkFlex(ShooterConstants.kLeaderMotorID, MotorType.kBrushless);
    followerShooterMotor =
      new CANSparkFlex(ShooterConstants.kFollowerMotorID, MotorType.kBrushless);
    feederMotor =
      new CANSparkMax(ShooterConstants.kFeederMotorID, MotorType.kBrushless);

    followerShooterMotor.follow(leaderShooterMotor, true);

    leaderShooterEncoder = leaderShooterMotor.getEncoder();
    feederMotor.getEncoder();
  }

  public void shooterStop() {
    leaderShooterMotor.set(0);
    //shooting = false;
  }

  public void feederStop() {
    feederMotor.set(0);
  }

  public void feed() {
    feederMotor.set(1);
  }

  public void shoot() {
    shooting = true;
  }

  public double getShooterSpeed() {
    return leaderShooterEncoder.getVelocity();
  }

  public void runAll(){
    leaderShooterMotor.set(1);
    feederMotor.set(1);
  }

  private void startState(){
    if (leaderShooterEncoder.getVelocity() >= ShooterConstants.kShooterTargetSpeed) {
      feedTimer.reset();
      feedTimer.start();

      feederMotor.set(1);
      state = ShooterState.FEEDING;
    } else {
      leaderShooterMotor.set(1);
    }
  }

  private void feedState(){
    if (feedTimer.get() >= ShooterConstants.kFeedDuration) {
      leaderShooterMotor.set(0);
      feederMotor.set(0);
      state = ShooterState.STOP;
      shooting = false;
    } else {
      leaderShooterMotor.set(1);
      feederMotor.set(1);
    }
  }

  private void stopState(){
    if (shooting) {
      leaderShooterMotor.set(1);
      state = ShooterState.STARTING;
    } else {
      leaderShooterMotor.set(0);
      followerShooterMotor.set(0);
    }
  }



  public void updateShooterState() {
    switch (state) {
      case STARTING:
      startState();
        break;

      case FEEDING:
      feedState();
        break;

      case STOP:
      stopState();
      break;
    }
  }


  @Override
  public void periodic() {
    updateShooterState();
    SmartDashboard.putNumber("counter", counter);
    SmartDashboard.putBoolean("isShooting",shooting);
    SmartDashboard.putNumber("Feed Timer", feedTimer.get());
    SmartDashboard.putNumber("Target Feed Time", ShooterConstants.kFeedDuration);
    SmartDashboard.putNumber("Leader Motor RPM", getShooterSpeed());
    SmartDashboard.putNumber("Motor Target Speed", ShooterConstants.kShooterTargetSpeed);
    SmartDashboard.putString("STATE", state.toString());
    SmartDashboard.putBoolean("got into feed state", gotIntoFeed);
  }

  public enum ShooterState {
    STARTING,
    FEEDING,
    STOP,
  }
}