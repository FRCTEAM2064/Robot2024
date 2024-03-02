package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.IntakeConstants;
import frc.robot.Constants.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkFlex leaderShooterMotor;
  private CANSparkFlex followerShooterMotor;
  private CANSparkMax feederMotor;

  private DigitalInput hasGamePieceSensor;

  private PowerDistribution pdh;

  private RelativeEncoder leaderShooterEncoder;
  public Timer feedTimer = new Timer();

  private ShooterState state = ShooterState.STOP;

  private boolean shooting = false;
  public boolean hasGamePiece = false;

  private int pulseCount = 0;

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

    pdh = new PowerDistribution(1, ModuleType.kRev);
    
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
    // if (hasGamePeice) {
    shooting = true;
    // }
  }

  public ShooterState getState() {
    return state;
  }


  public void updateHasGamePiece() {
    if(!hasGamePieceSensor.get()){
      hasGamePiece = true;
    }
    else{
      hasGamePiece = false;
    }
  }

  public void overrideHasGamePiece(){
    if(hasGamePiece == true){
      hasGamePiece = false;
    }
    else{
      hasGamePiece = true;
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
    if (shooting) {
      // feederEncoder.setPosition(0);
      // feederController.setReference(-0.1, ControlType.kPosition, 0);
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
    SmartDashboard.putBoolean("Shooter Has  Piece", hasGamePiece);
    SmartDashboard.putString("State", state.toString());
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed());
  }

  public void competitionValues() {
    SmartDashboard.putBoolean("ShooterPieces", hasGamePiece);

  }

  @Override
  public void periodic() {
    updateShooterState();
    updateHasGamePiece();
    // debugValues();
    resetGamePiece();
    competitionValues();
  }

  public enum ShooterState {
    STARTING,
    FEEDING,
    STOP,
  }
}
