package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.indexerPort, MotorType.kBrushless);
  private final DigitalInput frontBeambreak = new DigitalInput(Constants.intakeFrontBeambreak);
  private final DigitalInput backBeamBreak = new DigitalInput(Constants.intakeBackBeambreak);

  public final Trigger m_hasNote = new Trigger(backBeamBreak::get);
  public final Trigger m_intaking = new Trigger(frontBeambreak::get);

  public Index() {}

  public Command shoot() {
    return run(() -> m_motor.setVoltage(10)).until(m_hasNote.negate());
  }

  public Command intake() {
    return run(() -> m_motor.setVoltage(10)).until(m_hasNote);
  }

  public Command stop() {
    return runOnce(() -> m_motor.stopMotor());
  }
}
