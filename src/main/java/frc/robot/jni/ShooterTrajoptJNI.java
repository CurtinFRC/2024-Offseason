// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.jni;

import edu.wpi.first.util.RuntimeLoader;
import java.io.IOException;

public class ShooterTrajoptJNI {
  static RuntimeLoader<ShooterTrajoptJNI> loader = null;

  static {
    try {
      loader =
          new RuntimeLoader<>(
              "ShooterTrajoptJNI",
              RuntimeLoader.getDefaultExtractionRoot(),
              ShooterTrajoptJNI.class);
      loader.loadLibrary();
    } catch (IOException e) {
      e.printStackTrace();
      System.exit(1);
    }
  }

  /**
   * Calculate the optimal way to shoot a note to the speaker.
   *
   * @param javatraj The Java array which will be filled with the angular velocity, yaw and pitch.
   * @param x The X position of the robot.
   * @param y The Y position of the robot.
   * @param vel_x The X velocity of the robot.
   * @param vel_y The Y velocity of the robot.
   */
  public static native void calculateTrajectory(
      double[] javatraj, double x, double y, double vel_x, double vel_y);
}
