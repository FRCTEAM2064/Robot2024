package frc.robot.Subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class shooter {

  private CANSparkFlex leaderShooterMotor;
  private CANSparkFlex followerShooterMotor;
  private CANSparkMax feederMotor;

  private RelativeEncoder leaderShooterEncoder;
  private RelativeEncoder feederEncoder;
}
