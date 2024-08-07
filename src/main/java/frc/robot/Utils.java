// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DataLogManager;

/* Utilities used across the codebase */
public final class Utils {
  private Utils() {
    throw new IllegalStateException("This is a utility class and cannot be constructed.");
  }

  /**
   * Gets the current time in seconds.
   *
   * @return The current time in seconds.
   */
  public static double now() {
    return WPIUtilJNI.now() * 1e+6;
  }

  /**
   * A function to apply a 0.05 deadzone.
   *
   * @param value The value to apply a deadzone to.
   * @return The value with the applied deadzone.
   */
  public static double deadzone(double value) {
    return (Math.abs(value) > 0.05) ? value : 0;
  }

  /** A function to log CTRE {@link StatusCode}s to Datalog. */
  public static void logStatusCode(StatusCode code) {
    if (code.isOK()) {
      return;
    }

    if (code.isWarning()) {
      DataLogManager.log("Warning: " + code.getDescription());
    }

    if (code.isError()) {
      DataLogManager.log("Error: " + code.getDescription());
    }
  }
}
