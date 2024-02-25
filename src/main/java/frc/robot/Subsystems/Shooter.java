package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkFlex leaderShooterMotor;
  private CANSparkFlex followerShooterMotor;
  private CANSparkMax feederMotor;

  private PowerDistribution pdh;

  private RelativeEncoder leaderShooterEncoder;
  private RelativeEncoder feederEncoder;

  private SparkPIDController feederController;
  public Timer feedTimer = new Timer();

  private ShooterState state = ShooterState.STOP;

  private boolean shooting = false;
  public boolean hasGamePeice = false;

  public Shooter() {
    leaderShooterMotor =
      new CANSparkFlex(ShooterConstants.kLeaderMotorID, MotorType.kBrushless);
    followerShooterMotor =
      new CANSparkFlex(ShooterConstants.kFollowerMotorID, MotorType.kBrushless);
    feederMotor =
      new CANSparkMax(ShooterConstants.kFeederMotorID, MotorType.kBrushless);

    followerShooterMotor.follow(leaderShooterMotor, true);

    leaderShooterEncoder = leaderShooterMotor.getEncoder();
    feederMotor.setInverted(true);

    feederEncoder = feederMotor.getEncoder();
    feederController = feederMotor.getPIDController();

    feederController.setP(1);

    pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  public void shooterStop() {
    leaderShooterMotor.set(0);
  }

  public void feederStop() {
    feederMotor.set(0);
  }

  public void feed() {
    feederMotor.set(1.0);
  }

  public void intake() {
    feederMotor.set(1.0);
  }

  public void shoot() {
    if (hasGamePeice) {
      shooting = true;
    }
  }

  public ShooterState getState() {
    return state;
  }

  public void updateHasGamePiece() {
    if (
      pdh.getCurrent(ShooterConstants.kFeederMotorPDHPos) >
      ShooterConstants.kFeederMotorDrawLimit
    ) {
      hasGamePeice = true;
      feederMotor.set(0);
    }
  }

  public double getShooterSpeed() {
    return leaderShooterEncoder.getVelocity();
  }

  public void runAll() {
    leaderShooterMotor.set(1);
    feederMotor.set(1);
  }

  private void startState() {
    if (
      leaderShooterEncoder.getVelocity() >= ShooterConstants.kShooterTargetSpeed
    ) {
      feedTimer.reset();
      feedTimer.start();

      feederMotor.set(1.0);
      state = ShooterState.FEEDING;
    } else {
      leaderShooterMotor.set(1);
    }
  }

  private void feedState() {
    if (feedTimer.get() >= ShooterConstants.kFeedDuration) {
      leaderShooterMotor.set(0);
      feederMotor.set(0);
      feederMotor.setIdleMode(IdleMode.kCoast);
      state = ShooterState.STOP;
      shooting = false;
      hasGamePeice = false;
    } else {
      leaderShooterMotor.set(1);
      feederMotor.set(1);
    }
  }

  private void stopState() {
    if (shooting) {
      feederEncoder.setPosition(0);
      feederController.setReference(-0.1, ControlType.kPosition, 0);
      feederMotor.setIdleMode(IdleMode.kBrake);
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

  public void debugValues() {
    SmartDashboard.putNumber("Feed Timer", feedTimer.get());
    SmartDashboard.putBoolean("Shooting", shooting);
    SmartDashboard.putBoolean("Piece", hasGamePeice);
    SmartDashboard.putString("State", state.toString());
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
  }

  public void competitionValues() {}

  @Override
  public void periodic() {
    updateShooterState();
    updateHasGamePiece();
    debugValues();
    // competitionValues();
  }

  public enum ShooterState {
    STARTING,
    FEEDING,
    STOP,
  }
}
