package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public interface Auto {
  public void configureBindings();

  public Command followTrajectory();
}
