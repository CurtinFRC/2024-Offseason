package frc.robot;

import edu.wpi.first.util.WPIUtilJNI;
import java.lang.IllegalStateException;

public final class Utils {
  private Utils() {
      throw new IllegalStateException("This is a utility class and cannot be constructed.");
  }

  public static double now() {
    return WPIUtilJNI.now() * 1e+6;
  }
}
