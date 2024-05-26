package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private CANSparkMax m_upperMotor;
  private CANSparkMax m_lowerMotor;

  public Elevator(CANSparkMax upperMotor, CANSparkMax lowerMotor) {
    m_upperMotor = upperMotor;
    m_lowerMotor = lowerMotor;
  }

  public Command elevatorUp() {
    return Commands.run(() -> m_upperMotor.setVoltage(1))
      .withTimeout(2)
      .andThen(runOnce(() -> m_upperMotor.setVoltage(1)));
  }

  public Command elevatorDown() {
    return Commands.run(() -> m_lowerMotor.setVoltage(1))
      .withTimeout(2)
      .andThen(runOnce(() -> m_lowerMotor.setVoltage(1)));
  }
}
