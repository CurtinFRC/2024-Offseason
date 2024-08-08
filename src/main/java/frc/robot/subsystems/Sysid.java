// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/** Class that manages SysId commands and allows them to be executed sequentially. */
public class Sysid {
  private final List<Function<SysIdRoutine.Direction, Command>> commandFunctions =
      new ArrayList<>();

  /**
   * Adds a SysId command function to the list.
   *
   * @param commandFunction The command function to add.
   */
  public void add(Function<SysIdRoutine.Direction, Command> commandFunction) {
    commandFunctions.add(commandFunction);
  }

  /**
   * Adds multiple SysId command functions to the list.
   *
   * @param commands The list of command functions to add.
   */
  public void addAll(List<Function<SysIdRoutine.Direction, Command>> commands) {
    for (Function<SysIdRoutine.Direction, Command> command : commands) {
      commandFunctions.add(command);
    }
  }

  /**
   * Creates a command that executes all added SysId commands in the given direction sequentially.
   *
   * @param direction The direction for the SysId commands.
   * @return A sequential command group that executes all added SysId commands.
   */
  public Command getAllCommands(SysIdRoutine.Direction direction) {
    List<Command> commands = new ArrayList<>();
    for (Function<SysIdRoutine.Direction, Command> commandFunction : commandFunctions) {
      commands.add(commandFunction.apply(direction));
    }

    return new SequentialCommandGroup(commands.toArray(new Command[0]));
  }

  /**
   * Creates a command that executes all added SysId commands sequentially in both forward and
   * reverse directions.
   *
   * @return A sequential command group that executes all added SysId commands in both directions.
   */
  public Command getAllCommands() {
    return getAllCommands(SysIdRoutine.Direction.kForward)
        .andThen(getAllCommands(SysIdRoutine.Direction.kReverse));
  }
}
