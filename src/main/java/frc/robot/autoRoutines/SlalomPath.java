// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveForDistanceCommand;
import frc.robot.commands.drivetrain.TurnForAngleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlalomPath extends SequentialCommandGroup {
  /** Creates a new SlalomPath. */
  public SlalomPath(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForDistanceCommand(drivetrainSubsystem, 72, 0.5, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, 35, 0.5, 0.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 120, 0.5, 0.5),
      new TurnForAngleCommand(drivetrainSubsystem, 15, 0.5, 0.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 45, 0.25, 0.25),
      new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 50, 0.25, 0.25),
      new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5),
      new DriveForDistanceCommand(drivetrainSubsystem, 50, 0.25, 0.25),
      new TurnForAngleCommand(drivetrainSubsystem, -90, 0.5, 0.5)
    );
  }
}
