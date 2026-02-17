package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterAlignConstants {
  public static class Sim {
    public static final double minDist = 2;
    public static final double switchToHoodControlDist = 5;
    public static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap shooterHoodMap = new InterpolatingDoubleTreeMap();
    static {
      shooterRPMMap.put(2.0, 1408.525);
      shooterRPMMap.put(2.5, 1518.438);
      shooterRPMMap.put(3.0, 1650.196);
      shooterRPMMap.put(3.5, 1763.808);
      shooterRPMMap.put(4.0, 1894.0);
      shooterRPMMap.put(5.0, 2112.0);
      shooterRPMMap.put(5.5, 2112.0);
      shooterRPMMap.put(6.0, 2112.0);
    }
    static {
      shooterHoodMap.put(2.0, Units.degreesToRadians(66));
      shooterHoodMap.put(2.5, Units.degreesToRadians(66));
      shooterHoodMap.put(3.0, Units.degreesToRadians(66));
      shooterHoodMap.put(3.5, Units.degreesToRadians(66));
      shooterHoodMap.put(4.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.0, Units.degreesToRadians(66));
      shooterHoodMap.put(5.5, Units.degreesToRadians(61.05));
      shooterHoodMap.put(6.0, Units.degreesToRadians(55.28));
    }
  }
}
