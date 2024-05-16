package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** Our Arm Subsystem */
public class Arm extends SubsystemBase {
  private PIDController m_pid;
  private CANSparkMax m_primaryMotor;
  private CANSparkMax m_followerMotor;
  private RelativeEncoder m_encoder;

  /**
   * Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param primaryMotor The primary motor that controls the arm.
   * @param followerMotor The motor that follows the primary motor.
   */
  public Arm(CANSparkMax primaryMotor, CANSparkMax followerMotor) {
    m_primaryMotor = primaryMotor;
    m_followerMotor = followerMotor;
    m_followerMotor.follow(m_primaryMotor);

    m_encoder = m_primaryMotor.getEncoder();
    m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    m_pid.setTolerance(0.2, 0.5);
  }

  /** Achieves and maintains speed for the primary motor. */
  private Command achievePosition(double position) {
    return Commands.run(
        () -> m_primaryMotor.setVoltage(m_pid.calculate(m_encoder.getPosition(), position)));
  }

  /**
   * Moves the arm to the specified position then stops.
   *
   * @param position The desired position.
   * @return a {@link Command} to get to the desired position.
   */
  public Command moveToPosition(double position) {
    return achievePosition(position).until(m_pid::atSetpoint);
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
    return Commands.run(() -> m_primaryMotor.set(speed.getAsDouble()));
  }
}
