// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class LED extends SubsystemBase {
  private final PWM m_blinkin = new PWM(Constants.LEDport);
  private final StringPublisher m_activeState =
      NetworkTableInstance.getDefault().getTable("LEDs").getStringTopic("Active State").publish();

  public LED() {}

  public Command canShoot() {
    return defer(
        () -> {
          m_activeState.set("Shooter At Setpoint");
          return run(() -> m_blinkin.setSpeed(0.59)); // dark red
        });
  }

  public Command hasNote() {
    return defer(
        () -> {
          m_activeState.set("Has Note");
          return run(() -> m_blinkin.setSpeed(0.73)); // lime
        });
  }

  public Command hotpink() {
    return defer(
        () -> {
          m_activeState.set("Default");
          return run(() -> m_blinkin.setSpeed(0.57)); // hot pink
        });
  }
}
