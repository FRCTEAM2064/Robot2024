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
  private Timer feedTimer = new Timer();

  private ShooterState state = ShooterState.STOP;

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
  }

  public void feederStop() {
    feederMotor.set(0);
  }

  public void feed() {
    feederMotor.set(1);
  }

  public void shoot() {
    leaderShooterMotor.set(1);
    state = ShooterState.STARTING;
  }

  public void runAll(){
    leaderShooterMotor.set(1);
    feederMotor.set(1);
  }


  public void updateShooterState() {
    switch (state) {


      case STARTING:
        if (
          leaderShooterEncoder.getVelocity() >=
          ShooterConstants.kShooterTargetSpeed
        ) {
          feedTimer.reset();
          feedTimer.start();
          state = ShooterState.FEEDING;
          SmartDashboard.putNumber("InSTART", 1);
        }
        break;


      case FEEDING:
        SmartDashboard.putNumber("InFEED", 1);
        feed();
        if(feedTimer.get() > ShooterConstants.kFeedDuration) {
          state = ShooterState.STOP;
          feedTimer.stop();
          SmartDashboard.putNumber("InFEEDIF", 1);
        }
        break;


      case STOP:
        shooterStop();
        feederStop();
        break;
    }
  }

  public void periodic() {
    // updateShooterState();
    SmartDashboard.putNumber("Leader Motor RPM", leaderShooterEncoder.getVelocity());
    SmartDashboard.putString("STATE", state.toString());
  }

  public enum ShooterState {
    STARTING,
    FEEDING,
    STOP,
  }
}
