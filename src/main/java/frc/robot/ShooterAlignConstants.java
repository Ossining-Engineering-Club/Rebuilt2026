package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterAlignConstants {
  public static class Real {
    public static final double minDist = 0;
    public static final double latencyCompensationSeconds = 0.0;
    public static final double chassisSpeedsMultiplier = 1.0;
    public static final double distanceIncreaseScalar =
        0.1; // distance is multiplied by (1 + distanceIncreaseScalar *
    // shooterRadialVelocityRelativeToHub)
    public static final int maxTOFRecursions = 30;
    public static final double TOFRecursionTolerance =
        0.02; // unit of proportion error (range of [0,1])
    public static final double driveRotP = 0;
    public static final double driveRotI = 0;
    public static final double driveRotD = 0;
    public static final double driveRotA = 0;
    public static final double driveRotMaxAngularVelocity = Units.degreesToRadians(0);
    public static final double driveRotMaxAngularAcceleration = Units.degreesToRadians(0);
    public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterHoodMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();

    static {
      shooterRPMMap.put(0.0, 0.0);
    }

    static {
      shooterHoodMap.put(0.0, Units.degreesToRadians(0));
    }

    static {
      TOFMap.put(0.0, 0.0);
    }
  }

  public static class RealAuto {
    public static final double minDist = 0;
    public static final double latencyCompensationSeconds = 0.0;
    public static final double chassisSpeedsMultiplier = 1.0;
    public static final double distanceIncreaseScalar =
        0.1; // distance is multiplied by (1 + distanceIncreaseScalar *
    // shooterRadialVelocityRelativeToHub)
    public static final int maxTOFRecursions = 30;
    public static final double TOFRecursionTolerance =
        0.02; // unit of proportion error (range of [0,1])
    public static final double driveRotMaxAngularVelocity = Units.degreesToRadians(0);
    public static final double driveRotMaxAngularAcceleration = Units.degreesToRadians(0);
    public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterHoodMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();

    static {
      shooterRPMMap.put(0.0, 0.0);
    }

    static {
      shooterHoodMap.put(0.0, Units.degreesToRadians(0));
    }

    static {
      TOFMap.put(0.0, 0.0);
    }
  }

  public static class Sim {
    public static final double minDist = 2;
    public static final double latencyCompensationSeconds = 0.0;
    public static final double chassisSpeedsMultiplier = 0.6;
    public static final double distanceIncreaseScalar =
        0.1; // distance is multiplied by (1 + distanceIncreaseScalar *
    // shooterRadialVelocityRelativeToHub)
    public static final int maxTOFRecursions = 30;
    public static final double TOFRecursionTolerance =
        0.02; // unit of proportion error (range of [0,1])
    public static final double driveRotP = 8;
    public static final double driveRotI = 0;
    public static final double driveRotD = 0;
    public static final double driveRotA = 0.12;
    public static final double driveRotMaxAngularVelocity = Units.degreesToRadians(720);
    public static final double driveRotMaxAngularAcceleration = Units.degreesToRadians(360);
    public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterHoodMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();

    static {
      shooterRPMMap.put(2.0, 1408.525);
      shooterRPMMap.put(2.5, 1518.438);
      shooterRPMMap.put(3.0, 1650.196);
      shooterRPMMap.put(3.5, 1763.808);
      shooterRPMMap.put(4.0, 1894.0);
      shooterRPMMap.put(5.0, 2112.0);
      shooterRPMMap.put(5.5, 2301.0);
      shooterRPMMap.put(6.0, 2301.0);
      shooterRPMMap.put(7.0, 2301.0);
    }

    static {
      shooterHoodMap.put(2.0, Units.degreesToRadians(66));
      shooterHoodMap.put(2.5, Units.degreesToRadians(66));
      shooterHoodMap.put(3.0, Units.degreesToRadians(66));
      shooterHoodMap.put(3.5, Units.degreesToRadians(66));
      shooterHoodMap.put(4.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.5, Units.degreesToRadians(66));
      shooterHoodMap.put(6.0, Units.degreesToRadians(63.69));
      shooterHoodMap.put(7.0, Units.degreesToRadians(54.79));
    }

    static {
      TOFMap.put(2.0, 0.75);
      TOFMap.put(2.5, 0.93);
      TOFMap.put(3.0, 1.05);
      TOFMap.put(3.5, 1.17);
      TOFMap.put(4.0, 1.23);
      TOFMap.put(4.5, 1.37);
      TOFMap.put(5.0, 1.49);
      TOFMap.put(5.5, 1.62);
      TOFMap.put(6.0, 1.54);
      TOFMap.put(7.0, 1.37);
    }
  }

  public static class SimAuto {
    public static final double minDist = 2;
    public static final double latencyCompensationSeconds = 0.0;
    public static final double chassisSpeedsMultiplier = 0.6;
    public static final double distanceIncreaseScalar =
        0.1; // distance is multiplied by (1 + distanceIncreaseScalar *
    // shooterRadialVelocityRelativeToHub)
    public static final int maxTOFRecursions = 30;
    public static final double TOFRecursionTolerance =
        0.02; // unit of proportion error (range of [0,1])
    public static final double driveRotMaxAngularVelocity = Units.degreesToRadians(720);
    public static final double driveRotMaxAngularAcceleration = Units.degreesToRadians(360);
    public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterHoodMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();

    static {
      shooterRPMMap.put(2.0, 1408.525);
      shooterRPMMap.put(2.5, 1518.438);
      shooterRPMMap.put(3.0, 1650.196);
      shooterRPMMap.put(3.5, 1763.808);
      shooterRPMMap.put(4.0, 1894.0);
      shooterRPMMap.put(5.0, 2112.0);
      shooterRPMMap.put(5.5, 2301.0);
      shooterRPMMap.put(6.0, 2301.0);
      shooterRPMMap.put(7.0, 2301.0);
    }

    static {
      shooterHoodMap.put(2.0, Units.degreesToRadians(66));
      shooterHoodMap.put(2.5, Units.degreesToRadians(66));
      shooterHoodMap.put(3.0, Units.degreesToRadians(66));
      shooterHoodMap.put(3.5, Units.degreesToRadians(66));
      shooterHoodMap.put(4.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.5, Units.degreesToRadians(66));
      shooterHoodMap.put(6.0, Units.degreesToRadians(63.69));
      shooterHoodMap.put(7.0, Units.degreesToRadians(54.79));
    }

    static {
      TOFMap.put(2.0, 0.75);
      TOFMap.put(2.5, 0.93);
      TOFMap.put(3.0, 1.05);
      TOFMap.put(3.5, 1.17);
      TOFMap.put(4.0, 1.23);
      TOFMap.put(4.5, 1.37);
      TOFMap.put(5.0, 1.49);
      TOFMap.put(5.5, 1.62);
      TOFMap.put(6.0, 1.54);
      TOFMap.put(7.0, 1.37);
    }
  }
}
