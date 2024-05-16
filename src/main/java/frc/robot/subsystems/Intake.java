package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax m_motor;

  public Intake(CANSparkMax motor) {
    m_motor = motor;
  }

  public Command intake() {
    return Commands.run(
        () -> {
          m_motor.setVoltage(10);
        });
  }

  public Command outake() {
    return Commands.run(
        () -> {
          m_motor.setVoltage(-10);
        });
  }

  public Command pass() {
    return intake();
  }
}
