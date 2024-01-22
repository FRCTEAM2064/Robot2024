package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.Subsystems.Wrist;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WristTest {

  private Wrist wrist;

  @BeforeEach
  public void setup() {
    wrist = new Wrist();
  }

  @Test
  public void testWristMovement() {
    double targetAngle = 45.0;
    wrist.setWristAngle(targetAngle);
    wrist.updateWristState();
    assertEquals(targetAngle, wrist.getWristAngle(), 0.1);
  }

  @Test
  public void testWristStop() {
    wrist.stopWrist();
    assertEquals(0, wrist.getWristAngle(), 0.1);
  }
}
