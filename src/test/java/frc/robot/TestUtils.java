// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TestUtils {
  @BeforeEach
  void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
  }

  @Test
  void testNow() {
    SimHooks.stepTiming(3);
    assertEquals(Utils.now(), 3.0E12);
  }
}
