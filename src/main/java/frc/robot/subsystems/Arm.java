package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Our Arm Subsystem */
public class Arm extends SubsystemBase {
  public enum Setpoint {
    kAmp,
    kIntake,
    kSpeaker,
    kStowed
  }

  private PIDController m_pid;
  private CANSparkMax m_primaryMotor;
  private DutyCycleEncoder m_encoder;
  private ArmFeedforward m_feedforward;

  /**
   * Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param primaryMotor The primary motor that controls the arm.
   */
  public Arm(CANSparkMax primaryMotor) {
    m_primaryMotor = primaryMotor;

    m_encoder = new DutyCycleEncoder(Constants.armEncoderPort);
    m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    m_pid.setTolerance(0.2);
    m_feedforward =
        new ArmFeedforward(Constants.armS, Constants.armG, Constants.armV, Constants.armA);
  }

  /** Achieves and maintains speed for the primary motor. */
  private Command achievePosition(double position) {
    return Commands.run(
        () ->
            m_primaryMotor.setVoltage(
                -1
                    * (m_feedforward.calculate(position, (5676 / 250))
                        + m_pid.calculate(m_encoder.getAbsolutePosition() * 2 * 3.14, position))));
  }

  public Command stop() {
      return runOnce(() -> m_primaryMotor.set(0));
  }

  /**
   * Moves the arm to the specified position then stops.
   *
   * @param position The desired position.
   * @return a {@link Command} to get to the desired position.
   */
  private Command moveToPosition(double position) {
    return achievePosition(position)
        .until(() -> m_pid.atSetpoint() && m_encoder.getAbsolutePosition() * 2 * 3.14 == position);
  }

  /**
   * Holds the arm at the current position setpoint.
   *
   * @return A {@link Command} to hold the position at the setpoint.
   */
  public Command maintain() {
    return achievePosition(m_pid.getSetpoint());
  }

  /**
   * Checks if the Arm is at its setpoint and the loop is stable.
   *
   * @return A {@link Trigger} from the result.
   */
  public Trigger atSetpoint() {
    return new Trigger(() -> m_pid.atSetpoint());
  }

  /**
   * Allows for manual control of the arm using a joystick.
   *
   * @param speed The speed from the joystick input.
   * @return A {@link Command} to control the arm manually.
   */
  public Command manualControl(DoubleSupplier speed) {
    return Commands.run(() -> m_primaryMotor.set(speed.getAsDouble()), this);
  }

  /*
   * Makes the arm go to a setpoint from the {@link Setpoint} enum
   *
   * @param setpoint The setpoint to go to, a {@link Setpoint}
   * @return A {@link Command} to go to the setpoint
   */
  public Command goToSetpoint(Setpoint setpoint) {
    double position = 0;

    switch (setpoint) {
      case kAmp:
        position = 5.34;
        break;

      case kIntake:
        position = 3.7;
        break;

      case kSpeaker:
        position = 3.7;
        break;

      case kStowed:
        position = 3.7;
        break;
    }

    return moveToPosition(position);
  }
}
