// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  private final ExampleSubsystem subsystem;

  public ExampleCommand(ExampleSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    this.subsystem.motor.set(0);
  }

  @Override
  public void execute() {
    if (this.subsystem.limitSwitch.get()) {
      this.subsystem.motor.set(1);
    } else {
      this.subsystem.motor.set(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
