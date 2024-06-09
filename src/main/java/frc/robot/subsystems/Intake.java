package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DataLogManager; 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

public class Intake extends SubsystemBase {
  private CANSparkMax m_motor;
  private DataLog log = DataLogManager.getLog();
  private DoubleLogEntry intake_voltage_log = new DoubleLogEntry(log, "/intake/motor/voltage");
  public Intake(CANSparkMax motor) {
    m_motor = motor;
   intake_voltage_log.append(m_motor.getAppliedOutput());
  }

  public Command intake() {
    return Commands.run(() -> m_motor.setVoltage(10))
        .withTimeout(2)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command outake() {
    return Commands.run(() -> m_motor.setVoltage(-10))
        .withTimeout(4)
        .andThen(runOnce(() -> m_motor.setVoltage(0)));
  }

  public Command pass() {
    return intake();
  }

}
