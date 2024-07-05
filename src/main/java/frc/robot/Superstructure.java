// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.jni.ShooterTrajoptJNI;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

@SuppressWarnings("PMD.UnusedPrivateField")
public class Superstructure {
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Arm m_arm;
  private final CommandSwerveDrivetrain m_drivetrain;

  public Superstructure(
      Intake intake, Shooter shooter, Arm arm, CommandSwerveDrivetrain drivetrain) {
    m_intake = intake;
    m_shooter = shooter;
    m_arm = arm;
    m_drivetrain = drivetrain;
  }

  public Command shootFromRange() {
    var pose = m_drivetrain.getState().Pose;
    double[] traj = new double[3];

    ShooterTrajoptJNI.calculateTrajectory(traj, pose.getX(), pose.getY(), 0, 0);
    return Commands.parallel(
            m_arm.goToAngle(traj[2]),
            m_drivetrain.applyRequest(
                () ->
                    new SwerveRequest.FieldCentricFacingAngle()
                        .withTargetDirection(new Rotation2d(traj[1]))))
        .andThen(m_shooter.spinup(traj[0]))
        .andThen(m_shooter.maintain());
  }
}
