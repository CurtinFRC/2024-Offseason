// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import frc.robot.generated.TunerConstants;

public final class Constants {
  public static final int driverport = 0;
  public static final int codriverport = 1;

  public static final int shooterPort = 31;
  public static final int indexerPort = 99;
  public static final double shooterP = 0.5;
  public static final double shooterI = 0;
  public static final double shooterD = 0;

  public static final int climberPort = 32;
  public static final double climberP = 0.35;
  public static final double climberI = 0;
  public static final double climberD = 0;

  public static final int intakePort = 35;
  public static final int intakeFrontBeambreak = 99;
  public static final int intakeBackBeambreak = 99;

  public static final double armP = 21.0;
  public static final double armI = 0;
  public static final double armD = 0.015;
  public static final double armS = 0;
  public static final double armG = 0;
  public static final double armV = 0;
  public static final double armA = 0;
  public static final int armLeadPort = 21;
  public static final int armFollowerPort = 26;
  public static final int armEncoderPort = 3;

  public static final double DrivebaseMaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  public static final double DrivebaseMaxAngularRate = 1.5 * Math.PI;
}
