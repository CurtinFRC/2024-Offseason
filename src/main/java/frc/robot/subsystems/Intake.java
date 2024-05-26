package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  private CANSparkMax m_motor;
  private DigitalInput m_beamBreak = new DigitalInput(0);
  private Trigger m_trigger = new Trigger(m_beamBreak::get);

  public Intake(CANSparkMax motor) {
    m_motor = motor;
  }

  public Command intake() {
    return Commands.run(() -> m_motor.setVoltage(0))
        .withTimeout(4)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command outake() {
    return Commands.run(() -> m_motor.setVoltage(-10))
        // .withTimeout(4)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command pass() {
    return intake();
  }
}
