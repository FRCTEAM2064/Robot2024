package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class FieldConstants {

  public static final Map<Integer, Pose3d> aprilTags = Map.of(
    1,
    new Pose3d(
      Units.inchesToMeters(610.77),
      Units.inchesToMeters(42.19),
      Units.inchesToMeters(18.22),
      new Rotation3d(0.0, 0.0, Math.PI)
    ),
    2,
    new Pose3d(
      Units.inchesToMeters(610.77),
      Units.inchesToMeters(108.19),
      Units.inchesToMeters(18.22),
      new Rotation3d(0.0, 0.0, Math.PI)
    ),
    3,
    new Pose3d(
      Units.inchesToMeters(610.77),
      Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
      Units.inchesToMeters(18.22),
      new Rotation3d(0.0, 0.0, Math.PI)
    ),
    4,
    new Pose3d(
      Units.inchesToMeters(636.96),
      Units.inchesToMeters(265.74),
      Units.inchesToMeters(27.38),
      new Rotation3d(0.0, 0.0, Math.PI)
    ),
    5,
    new Pose3d(
      Units.inchesToMeters(14.25),
      Units.inchesToMeters(265.74),
      Units.inchesToMeters(27.38),
      new Rotation3d()
    ),
    6,
    new Pose3d(
      Units.inchesToMeters(40.45),
      Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
      Units.inchesToMeters(18.22),
      new Rotation3d()
    ),
    7,
    new Pose3d(
      Units.inchesToMeters(40.45),
      Units.inchesToMeters(108.19),
      Units.inchesToMeters(18.22),
      new Rotation3d()
    ),
    8,
    new Pose3d(
      Units.inchesToMeters(40.45),
      Units.inchesToMeters(42.19),
      Units.inchesToMeters(18.22),
      new Rotation3d()
    )
  );
}
