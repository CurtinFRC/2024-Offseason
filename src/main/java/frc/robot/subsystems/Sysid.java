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
import java.util.function.Supplier;

public class Sysid {
    private final List<Function<SysIdRoutine.Direction, Command>> commandFunctions = new ArrayList<>();

    public void add(Function<SysIdRoutine.Direction, Command> commandFunction) {
        commandFunctions.add(commandFunction);
    }

    public void addAll(List<Function<SysIdRoutine.Direction, Command>> commands) {
        for (Function<SysIdRoutine.Direction, Command> command : commands) {
            commandFunctions.add(command);
        }
    }


    public Command getAllCommands(SysIdRoutine.Direction direction) {
        List<Command> commands = new ArrayList<>();
        for (Function<SysIdRoutine.Direction, Command> commandFunction : commandFunctions) {
            commands.add(commandFunction.apply(direction));
        }

        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

  public Command getAllCommands() {
    return getAllCommands(SysIdRoutine.Direction.kForward).andThen(getAllCommands(SysIdRoutine.Direction.kReverse));
  }
}
