// taken from https://github.com/Mechanical-Advantage/URCL/pull/7
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.urcl.URCLJNI;

/**
 *
 *
 * <h2>URCL (Unofficial REV-Compatible Logger)</h2>
 *
 * This unofficial logger enables automatic capture of CAN traffic from REV motor controllers to
 * NetworkTables, viewable using AdvantageScope. See the corresponding <a href=
 * "https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/REV-LOGGING.md">
 * AdvantageScope documentation</a> for more details.
 *
 * <p><b>As this library is not an official REV tool, support queries should be directed to the URCL
 * <a href="https://github.com/Mechanical-Advantage/URCL/issues">issues page</a> or
 * software@team6328.org rather than REV's support contact.</b>
 */
@SuppressWarnings("PMD")
public class URCL {
  private static final double period = 0.02;

  private static boolean running = false;
  private static ByteBuffer persistentBuffer;
  private static ByteBuffer periodicBuffer;
  private static ByteBuffer aliasesBuffer;
  private static RawPublisher persistentPublisher;
  private static RawPublisher periodicPublisher;
  private static RawPublisher aliasesPublisher;
  private static Notifier notifier;
  private static final DataLog datalog = DataLogManager.getLog();
  private static RawLogEntry persistentLogEntry =
      new RawLogEntry(datalog, "URCL/Raw/Persistent", "", "URCLr2_persistent");
  private static RawLogEntry periodicLogEntry =
      new RawLogEntry(datalog, "/URCL/Raw/Periodic", "", "URCLr2_periodic");
  private static RawLogEntry aliasLogEntry =
      new RawLogEntry(datalog, "/URCL/Raw/Aliases", "", "URCLr2_aliases");

  /**
   * Start capturing data from REV motor controllers to NetworkTables. This method should only be
   * called once.
   */
  public static void start() {
    start(Map.of());
  }

  /**
   * Start capturing data from REV motor controllers to NetworkTables. This method should only be
   * called once.
   *
   * @param withNT Whether or not to run with NetworkTables.
   */
  public static void start(boolean withNT) {
    start(Map.of(), withNT);
  }

  /**
   * Start capturing data from REV motor controllers to NetworkTables. This method should only be
   * called once.
   *
   * @param aliases The set of aliases mapping CAN IDs to names.
   */
  public static void start(Map<Integer, String> aliases) {
    start(aliases, true);
  }

  /**
   * Start capturing data from REV motor controllers to NetworkTables. This method should only be
   * called once.
   *
   * @param aliases The set of aliases mapping CAN IDs to names.
   * @param withNT Whether or not to run with NetworkTables.
   */
  public static void start(Map<Integer, String> aliases, boolean withNT) {
    if (running) {
      DriverStation.reportError("URCL cannot be started multiple times", true);
      return;
    }
    running = true;

    // Update aliases buffer
    updateAliasesBuffer(aliases);

    // Start driver
    URCLJNI.start();
    persistentBuffer = URCLJNI.getPersistentBuffer();
    periodicBuffer = URCLJNI.getPeriodicBuffer();
    persistentBuffer.order(ByteOrder.LITTLE_ENDIAN);
    periodicBuffer.order(ByteOrder.LITTLE_ENDIAN);

    if (withNT) {
      // Start publishers
      persistentPublisher =
          NetworkTableInstance.getDefault()
              .getRawTopic("/URCL/Raw/Persistent")
              .publish("URCLr2_persistent");
      periodicPublisher =
          NetworkTableInstance.getDefault()
              .getRawTopic("/URCL/Raw/Periodic")
              .publish("URCLr2_periodic");
      aliasesPublisher =
          NetworkTableInstance.getDefault()
              .getRawTopic("/URCL/Raw/Aliases")
              .publish("URCLr2_aliases");
      notifier =
          new Notifier(
              () -> {
                var data = getData();
                persistentPublisher.set(data[0]);
                periodicPublisher.set(data[1]);
                aliasesPublisher.set(data[2]);
              });
    } else {
      notifier =
          new Notifier(
              () -> {
                var data = getData();
                persistentLogEntry.append(data[0]);
                periodicLogEntry.append(data[1]);
                aliasLogEntry.append(data[2]);
              });
    }

    notifier.setName("URCL");
    notifier.startPeriodic(period);
  }

  /**
   * Start capturing data from REV motor controllers to an external framework like <a href=
   * "https://github.com/Mechanical-Advantage/AdvantageKit">AdvantageKit</a>. This method should
   * only be called once.
   *
   * @return The log supplier, to be called periodically
   */
  public static Supplier<ByteBuffer[]> startExternal() {
    return startExternal(Map.of());
  }

  /**
   * Start capturing data from REV motor controllers to an external framework like <a href=
   * "https://github.com/Mechanical-Advantage/AdvantageKit">AdvantageKit</a>. This method should
   * only be called once.
   *
   * @param aliases The set of aliases mapping CAN IDs to names.
   * @return The log supplier, to be called periodically
   */
  public static Supplier<ByteBuffer[]> startExternal(Map<Integer, String> aliases) {
    if (running) {
      DriverStation.reportError("URCL cannot be started multiple times", true);
      ByteBuffer[] emptyOutput =
          new ByteBuffer[] {ByteBuffer.allocate(0), ByteBuffer.allocate(0), ByteBuffer.allocate(0)};
      return () -> emptyOutput;
    }
    running = true;

    // Update aliases buffer
    updateAliasesBuffer(aliases);

    // Start driver
    URCLJNI.start();
    persistentBuffer = URCLJNI.getPersistentBuffer();
    periodicBuffer = URCLJNI.getPeriodicBuffer();
    persistentBuffer.order(ByteOrder.LITTLE_ENDIAN);
    periodicBuffer.order(ByteOrder.LITTLE_ENDIAN);
    return URCL::getData;
  }

  /** Fills the alias data into the aliases buffer as JSON. */
  private static void updateAliasesBuffer(Map<Integer, String> aliases) {
    StringBuilder aliasesBuilder = new StringBuilder();
    aliasesBuilder.append("{");
    boolean firstEntry = true;
    for (Map.Entry<Integer, String> entry : aliases.entrySet()) {
      if (!firstEntry) {
        aliasesBuilder.append(",");
      }
      firstEntry = false;
      aliasesBuilder.append("\"");
      aliasesBuilder.append(entry.getKey().toString());
      aliasesBuilder.append("\":\"");
      aliasesBuilder.append(entry.getValue());
      aliasesBuilder.append("\"");
    }
    aliasesBuilder.append("}");
    aliasesBuffer = Charset.forName("UTF-8").encode(aliasesBuilder.toString());
  }

  /** Refreshes and reads all data from the queues. */
  private static ByteBuffer[] getData() {
    URCLJNI.read();
    int persistentSize = persistentBuffer.getInt(0);
    int periodicSize = periodicBuffer.getInt(0);
    return new ByteBuffer[] {
      persistentBuffer.slice(4, persistentSize),
      periodicBuffer.slice(4, periodicSize),
      aliasesBuffer
    };
  }
}
