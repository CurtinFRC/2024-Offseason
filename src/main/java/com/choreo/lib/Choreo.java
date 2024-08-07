// Copyright (c) Choreo contributors

package com.choreo.lib;

import com.google.gson.Gson;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Utilities to load and follow ChoreoTrajectories */
public final class Choreo {
  private static final Gson gson = new Gson();
  private static Timer timer = new Timer();

  private static ChoreoTrajectory emptyTraj = new ChoreoTrajectory();
  private static ChoreoTrajectory currentTraj = emptyTraj;
  private static boolean started;

  /** Default constructor. */
  private Choreo() {}

  /**
   * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
   * src/main/deploy/choreo/[trajName].traj .
   *
   * @param trajName the path name in Choreo, which matches the file name in the deploy directory.
   *     Do not include ".traj" here.
   * @return the loaded trajectory, or null if the trajectory could not be loaded.
   */
  public static ChoreoTrajectory getTrajectory(String trajName) {
    var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
    var traj_file = new File(traj_dir, trajName + ".traj");

    return loadFile(traj_file);
  }

  /**
   * Loads the split parts of the specified trajectory. Fails and returns null if any of the parts
   * could not be loaded.
   *
   * <p>This method determines the number of parts to load by counting the files that match the
   * pattern "trajName.X.traj", where X is a string of digits. Let this count be N. It then attempts
   * // * to load "trajName.1.traj" through "trajName.N.traj", consecutively counting up. If any of
   * these files cannot be loaded, the method returns null.
   *
   * @param trajName The path name in Choreo for this trajectory.
   * @return The ArrayList of segments, in order, or null.
   */
  @SuppressWarnings("PMD.ReturnEmptyCollectionRatherThanNull")
  public static ArrayList<ChoreoTrajectory> getTrajectoryGroup(String trajName) {
    // Count files matching the pattern for split parts.
    var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
    File[] files =
        traj_dir.listFiles((file) -> file.getName().matches(trajName + "\\.\\d+\\.traj"));
    if (files == null) {
      DriverStation.reportError("ChoreoLib: Cannot get files for trajectory group", false);
      return null;
    }
    int segmentCount = files.length;
    // Try to load the segments.
    var trajs = new ArrayList<ChoreoTrajectory>();
    for (int i = 1; i <= segmentCount; ++i) {
      File traj = new File(traj_dir, String.format("%s.%d.traj", trajName, i));
      ChoreoTrajectory trajectory = loadFile(traj);
      if (trajectory == null) {
        DriverStation.reportError("ChoreoLib: Missing segments for path group " + trajName, false);
        return null;
      }
      trajs.add(trajectory);
    }

    return trajs;
  }

  @SuppressWarnings({"PMD.AvoidCatchingGenericException", "PMD.AvoidFileStream"})
  private static ChoreoTrajectory loadFile(File path) {
    try {
      var reader = new BufferedReader(new FileReader(path));
      ChoreoTrajectory traj = gson.fromJson(reader, ChoreoTrajectory.class);

      return traj;
    } catch (Exception ex) {
      DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
    }
    return null;
  }

  /**
   * Create a command to follow a Choreo path.
   *
   * @param trajectory The trajectory to follow. Use Choreo.getTrajectory(String trajName) to load
   *     this from the deploy directory.
   * @param poseSupplier A function that returns the current field-relative pose of the robot.
   * @param xController A PIDController for field-relative X translation (input: X error in meters,
   *     output: m/s).
   * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
   *     output: m/s).
   * @param rotationController A PIDController for robot rotation (input: heading error in radians,
   *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
   *     ChoreoLib.
   * @param outputChassisSpeeds A function that consumes the target robot-relative chassis speeds
   *     and commands them to the robot.
   * @param mirrorTrajectory If this returns true, the path will be mirrored to the opposite side,
   *     while keeping the same coordinate system origin. This will be called every loop during the
   *     command.
   * @param requirements The subsystem(s) to require, typically your drive subsystem only.
   * @return A command that follows a Choreo path.
   */
  public static Command choreoSwerveCommand(
      ChoreoTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier mirrorTrajectory,
      Subsystem... requirements) {
    return choreoSwerveCommand(
        trajectory,
        poseSupplier,
        choreoSwerveController(xController, yController, rotationController),
        outputChassisSpeeds,
        mirrorTrajectory,
        requirements);
  }

  /**
   * Create a command to follow a Choreo path.
   *
   * @param trajectory The trajectory to follow. Use Choreo.getTrajectory(String trajName) to load
   *     this from the deploy directory.
   * @param poseSupplier A function that returns the current field-relative pose of the robot.
   * @param controller A ChoreoControlFunction to follow the current trajectory state. Use
   *     ChoreoCommands.choreoSwerveController(PIDController xController, PIDController yController,
   *     PIDController rotationController) to create one using PID controllers for each degree of
   *     freedom. You can also pass in a function with the signature (Pose2d currentPose,
   *     ChoreoTrajectoryState referenceState) -&gt; ChassisSpeeds to implement a custom follower
   *     (i.e. for logging).
   * @param outputChassisSpeeds A function that consumes the target robot-relative chassis speeds
   *     and commands them to the robot.
   * @param mirrorTrajectory If this returns true, the path will be mirrored to the opposite side,
   *     while keeping the same coordinate system origin. This will be called every loop during the
   *     command.
   * @param requirements The subsystem(s) to require, typically your drive subsystem only.
   * @return A command that follows a Choreo path.
   */
  public static Command choreoSwerveCommand(
      ChoreoTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      ChoreoControlFunction controller,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier mirrorTrajectory,
      Subsystem... requirements) {
    return new FunctionalCommand(
        () -> {
          timer.restart();
          currentTraj = trajectory;
        },
        () -> {
          ;
          outputChassisSpeeds.accept(
              controller.apply(
                  poseSupplier.get(),
                  trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean())));
        },
        (interrupted) -> {
          timer.stop();
          currentTraj = emptyTraj;

          if (interrupted) {
            outputChassisSpeeds.accept(new ChassisSpeeds());
          } else {
            outputChassisSpeeds.accept(trajectory.getFinalState().getChassisSpeeds());
          }
        },
        () -> timer.hasElapsed(trajectory.getTotalTime()),
        requirements);
  }

  /**
   * Creates a control function for following a ChoreoTrajectoryState.
   *
   * @param xController A PIDController for field-relative X translation (input: X error in meters,
   *     output: m/s).
   * @param yController A PIDController for field-relative Y translation (input: Y error in meters,
   *     output: m/s).
   * @param rotationController A PIDController for robot rotation (input: heading error in radians,
   *     output: rad/s). This controller will have its continuous input range set to -pi..pi by
   *     ChoreoLib.
   * @return A ChoreoControlFunction to track ChoreoTrajectoryStates. This function returns
   *     robot-relative ChassisSpeeds.
   */
  public static ChoreoControlFunction choreoSwerveController(
      PIDController xController, PIDController yController, PIDController rotationController) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    return (pose, referenceState) -> {
      double xFF = referenceState.velocityX;
      double yFF = referenceState.velocityY;
      double rotationFF = referenceState.angularVelocity;

      double xFeedback = xController.calculate(pose.getX(), referenceState.x);
      double yFeedback = yController.calculate(pose.getY(), referenceState.y);
      double rotationFeedback =
          rotationController.calculate(pose.getRotation().getRadians(), referenceState.heading);

      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());
    };
  }

  private static boolean onTrajectory(String trajName) {
    return currentTraj.equals(getTrajectory(trajName));
  }

  /**
   * Returns a Trigger which fires if the robot is currently on a given ChoreoTrajectory.
   *
   * @param trajName The file name (without the .traj) of the given trajectory.
   * @return A Trigger which activates if the robot is on the trajectory trajName.
   */
  public static Trigger trajTrigger(String trajName) {
    return new Trigger(() -> Choreo.onTrajectory(trajName));
  }

  /**
   * Returns a Trigger which fires when the robot hits an event marker, then stops when the robot
   * starts on a different trajectory.
   *
   * @param trajName The file name (without the .traj) of the given trajectory.
   * @param offset The time between when the inputted trajectory is started and when the event
   *     trigger should fire.
   * @return A Trigger which activates when the robot hits an event marker.
   */
  public static Trigger pointTrigger(String trajName, double offset) {
    var timer = new Timer();
    return new Trigger(
        () -> {
          if (onTrajectory(trajName) && !started) {
            started = true;
            timer.restart();
          }
          return started && timer.hasElapsed(offset) && onTrajectory(trajName);
        });
  }

  /**
   * Returns a Trigger which activates at a certain time since starting on a ChoreoTrajectory, then
   * deactivates at a point in time after that. The generated Triggers will not stop firing even
   * when the robot starts on another ChoreoTrajectory.
   *
   * @param trajName The file name (without the .traj) of the given trajectory.
   * @param risingEdge The time since the robot has started on the trajectory trajName, in seconds,
   *     that the trigger should begin to fire.
   * @param fallingEdge The time since the robot has started on the trajectory trajName, in seconds,
   *     that the trigger should stop firing. fallingEdge should be a greater number than
   *     risingEdge, or the Trigger will not fire.
   * @return A Trigger which activates if the robot is on the trajectory trajName.
   */
  public static Trigger spanTrigger(String trajName, double risingEdge, double fallingEdge) {
    var timer = new Timer();
    return new Trigger(
        () -> {
          if (onTrajectory(trajName) && !started) {
            started = true;
            timer.restart();
          }
          return started && timer.hasElapsed(risingEdge) && !timer.hasElapsed(fallingEdge);
        });
  }

  /**
   * Returns a Trigger which activates for a certain period of time after the robot hits an event
   * marker. The created Triggers will not stop firing even when the robot starts on another
   * ChoreoTrajectory.
   *
   * @param trajName The file name (without the .traj) of the given trajectory.
   * @param offset The time in seconds between when the inputted trajectory is started and when the
   *     event trigger should fire.
   * @param length The duration of the trigger in seconds, from start to finish.
   * @return A Trigger which activates if the robot is on the trajectory trajName.
   */
  public static Trigger spotTrigger(String trajName, double offset, double length) {
    var timer = new Timer();
    return new Trigger(
        () -> {
          if (onTrajectory(trajName) && !started) {
            started = true;
            timer.restart();
          }
          return started
              && timer.hasElapsed(offset)
              && onTrajectory(trajName)
              && !timer.hasElapsed(offset + length);
        });
  }

  /**
   * Returns a trigger, which activates at the specified event marker's start time, and ends at the
   * event marker's end time.
   *
   * @param eventName The name of the event marker.
   * @return A trigger which activates during an event marker.
   */
  public static Trigger event(String eventName) {
    return new Trigger(
        () ->
            timer.hasElapsed(currentTraj.markerFromName(eventName).getStartTime())
                && !timer.hasElapsed(currentTraj.markerFromName(eventName).getEndTime()));
  }
}
