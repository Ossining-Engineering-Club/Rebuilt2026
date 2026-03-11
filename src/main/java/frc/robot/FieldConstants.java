package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLengthMeters = Units.inchesToMeters(651.2); // x
  public static final double fieldWidthMeters = Units.inchesToMeters(317.7); // y

  public static final double fieldCenterY = fieldWidthMeters / 2.0;

  // bump: bounding box coordinates are the points at which the wheels of the robot just barely
  // touch the bump
  public static final double blueBumpMinX = 3.676;
  public static final double blueBumpMaxX = 5.577;
  public static final double blueBumpMinY = 1.974;
  public static final double blueBumpMaxY = 6.099;
  public static final double redBumpMinX = 10.966;
  public static final double redBumpMaxX = 12.860;
  public static final double redBumpMinY = 1.974;
  public static final double redBumpMaxY = 6.099;

  public static final double blueBumpCenterX = Units.inchesToMeters(182.11);
  public static final double redBumpCenterX = Units.inchesToMeters(651.22 - 182.11);

  public static final Translation2d blueHub =
      new Translation2d(Inches.of(182.11), Inches.of(158.84));
  public static final Translation2d redHub =
      new Translation2d(Inches.of(469.11), Inches.of(158.84));

  public static final Translation2d bluePassingTopTarget =
      new Translation2d(blueBumpCenterX - 1.5, fieldCenterY + Units.inchesToMeters(91.17));
  public static final Translation2d bluePassingBottomTarget =
      new Translation2d(blueBumpCenterX - 1.5, fieldCenterY - Units.inchesToMeters(91.17));
  public static final Translation2d redPassingTopTarget =
      new Translation2d(redBumpCenterX + 1.5, fieldCenterY + Units.inchesToMeters(91.17));
  public static final Translation2d redPassingBottomTarget =
      new Translation2d(redBumpCenterX + 1.5, fieldCenterY - Units.inchesToMeters(91.17));
}
