// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Our Crescendo climber Subsystem */
public class Climber extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.climberPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final PIDController m_pid =
      new PIDController(Constants.climberP, Constants.climberI, Constants.climberD);

  private final DoubleLogEntry log_pid_output =
      new DoubleLogEntry(DataLogManager.getLog(), "/climber/pid/output");

  /** Creates a new {@link Climber} {@link edu.wpi.first.wpilibj2.command.Subsystem}. */
  public Climber() {}

  /**
   * Climb the robot!
   *
   * @return A {@link Command} to climb the robot.
   */
  public Command climb() {
    return run(
        () -> {
          var output =
              m_pid.calculate(Units.rotationsToRadians(m_encoder.getPosition() * -1), -Math.PI);
          log_pid_output.append(output);
          m_motor.setVoltage(output);
        });
  }
}
