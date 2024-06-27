// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Consumer;

/**
 * A Command to follow a Choreo trajectory. Uses an LTV Unicycle controller.
 *
 * @apiNote Don't use this class directly instead use the factory provided by {@link
 *     frc.robot.CommandSwerveDrivetrain}.
 */
public class FollowTrajectory extends Command {
  private Timer m_timer;
  private LTVUnicycleController m_controller;
  private CommandSwerveDrivetrain m_drivetrain;
  private ChoreoTrajectory m_trajectory;
  private ChassisSpeeds m_lastSpeeds;
  private boolean m_isRed;
  private Consumer<ChassisSpeeds> m_output;

  /**
   * Creates a new FollowTrajectory Command.
   *
   * @param drivetrain The drivebase.
   * @param trajectoryName The name of the trajectory to follow.
   * @param isRed Whether or not the path should be from a red perspective.
   * @param output The function that controls the drivebase.
   */
  public FollowTrajectory(
      CommandSwerveDrivetrain drivetrain,
      String trajectoryName,
      boolean isRed,
      Consumer<ChassisSpeeds> output) {
    super();
    m_drivetrain = drivetrain;
    m_trajectory = Choreo.getTrajectory(trajectoryName);
    m_isRed = isRed;
    m_output = output;
    m_timer = new Timer();
    m_lastSpeeds = new ChassisSpeeds();
    m_controller = new LTVUnicycleController(0.02, Constants.DrivebaseMaxSpeed);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    var speed = m_drivetrain.getState().speeds;
    var speedsDiff = speed.minus(m_lastSpeeds);
    var acceleration =
        Math.hypot(speedsDiff.vxMetersPerSecond, speedsDiff.vyMetersPerSecond) / 0.02;
    var state = m_trajectory.sample(m_timer.get(), m_isRed);
    var desiredSpeeds =
        m_controller.calculate(m_drivetrain.getState().Pose, state.toTrajectoryState(acceleration));
    m_output.accept(desiredSpeeds);
    m_lastSpeeds = speed;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_output.accept(new ChassisSpeeds());
    } else {
      m_output.accept(m_trajectory.getFinalState().getChassisSpeeds());
    }

    m_timer.stop();
    m_trajectory = new ChoreoTrajectory();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTime());
  }

  /**
   * Gets the Trigger for an event marker.
   *
   * @param eventName The name of the marker.
   * @return The trigger for the event marker.
   */
  public Trigger event(String eventName) {
    return new Trigger(
        () ->
            m_timer.hasElapsed(m_trajectory.markerFromName(eventName).startTime())
                && !m_timer.hasElapsed(m_trajectory.markerFromName(eventName).endTime()));
  }
}
