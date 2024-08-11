// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package edu.wpi.first.wpilibj2.command;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * CommandRobot is a wrapper over TimedRobot designed to work well with Command based programming.
 *
 * <p>The CommandRobot class is intended to be subclassed by a user creating a robot program.
 */
public class CommandRobot extends TimedRobot {
  /** The CommandScheduler instance. */
  protected CommandScheduler m_scheduler = CommandScheduler.getInstance();

  /** The autonomous chooser. */
  protected SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private Command m_autonomousCommand;

  /** Constructor for CommandRobot. */
  protected CommandRobot() {
    this(kDefaultPeriod);
  }

  /**
   * Constructor for CommandRobot.
   *
   * @param period Period in seconds.
   */
  protected CommandRobot(double period) {
    super(period);
    addPeriodic(() -> m_scheduler.run(), kDefaultPeriod);
    m_autoChooser.setDefaultOption("No Auto Configured", Commands.print("No autos configured."));
  }

  @Override
  public final void robotInit() {}

  @Override
  public final void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_scheduler.schedule(m_autonomousCommand);
    }
  }

  @Override
  public final void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public final void teleopInit() {
    if (m_autonomousCommand != null) {
      m_scheduler.cancel(m_autonomousCommand);
    }
  }

  @Override
  public final void teleopPeriodic() {}

  @Override
  public final void teleopExit() {}

  @Override
  public final void disabledInit() {}

  @Override
  public final void disabledPeriodic() {}

  @Override
  public final void disabledExit() {}
}
