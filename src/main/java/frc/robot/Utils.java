// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import edu.wpi.first.util.WPIUtilJNI;

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
}
