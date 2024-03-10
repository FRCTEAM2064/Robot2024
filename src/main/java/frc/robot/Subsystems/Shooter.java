package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.fasterxml.jackson.databind.ser.std.ToEmptyObjectSerializer;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final CANSparkFlex leaderShooterMotor;
  private final CANSparkFlex followerShooterMotor;
  private final CANSparkMax feederMotor;

  private final DigitalInput hasGamePieceSensor;

    private final RelativeEncoder leaderShooterEncoder;
  public Timer feedTimer = new Timer();
  public Timer shootTimer = new Timer();

  public ShooterState state = ShooterState.STOP;

  // private boolean shooting = false;
  public boolean hasGamePiece = false;

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

    feederMotor.getEncoder();

    feederMotor.setSmartCurrentLimit(20);

      hasGamePieceSensor = new DigitalInput(ShooterConstants.kHasGamePieceDIO);

  }

  public void shooterStop() {
    leaderShooterMotor.set(0);
  }

  public void feederStop() {
    feederMotor.set(0);
  }

  public void feed() {
    feederMotor.set(0.2);
  }
  
  public void cleanFeed() {
    feederMotor.set(0.05);
  }

  public void intake() {
    feederMotor.set(1.0);
  }

  public void shoot() {
    // shooting = true;
    state = ShooterState.STARTING;
    shootTimer.reset();
    shootTimer.start();
  }

  public ShooterState getState() {
    return state;
  }


  public void updateHasGamePiece() {
      hasGamePiece = !hasGamePieceSensor.get();
  }

  public void overrideHasGamePiece(){
      hasGamePiece = !hasGamePiece;
  }

  public double getShooterSpeed() {
    return leaderShooterEncoder.getVelocity();
  }

  public void runAll() {
    leaderShooterMotor.set(.4);
    feederMotor.set(1);
  }

  private void startState() {
    if (
      leaderShooterEncoder.getVelocity() >= ShooterConstants.kShooterTargetSpeed || shootTimer.get() >= 1.5
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
      // shooting = false;
      hasGamePiece = false;
    } else {
      leaderShooterMotor.set(1);
      feederMotor.set(1);
    }
  }

  private void resetGamePiece(){
    if(feedTimer.get() > 5){
      hasGamePiece = false;
      feedTimer.stop();
      feedTimer.reset();
    }
  }

  private void stopState() {
    // if (shooting) {
    //     leaderShooterMotor.set(1);
    //   state = ShooterState.STARTING;
    // } else {
    //   leaderShooterMotor.set(0);
    //   followerShooterMotor.set(0);
    // }
    leaderShooterMotor.set(0);
    feederMotor.set(0);
  }

  private void ampState(){
    feedTimer.start();

    if (feedTimer.get() >= 3) {
      state = ShooterState.STOP;
    } else {
    leaderShooterMotor.set(.1);
    feederMotor.set(.2);
    }
  }

  public void setStateAmp(){
    feedTimer.reset();
    state = ShooterState.AMP;
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
      case AMP:
        ampState();
        break;
    }
  }

  public void debugValues() {
    SmartDashboard.putNumber("Feed Timer", feedTimer.get());
    // SmartDashboard.putBoolean("Shooting", shooting);
    SmartDashboard.putBoolean("Shooter Has  Piece", hasGamePiece);
    SmartDashboard.putString("Shooter State", state.toString());
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
  }

  public void competitionValues() {
    SmartDashboard.putBoolean("ShooterPieces", hasGamePiece);
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());


  }

  @Override
  public void periodic() {
    updateShooterState();
    updateHasGamePiece();
    debugValues();
    resetGamePiece();
    // competitionValues();
  }

  public enum ShooterState {
    STARTING,
    FEEDING,
    STOP,
    AMP,
  }
}
