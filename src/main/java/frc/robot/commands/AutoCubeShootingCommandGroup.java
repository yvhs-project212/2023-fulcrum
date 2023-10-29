// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCubeShootingCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoCubeShootingCommandGroup. */
  public AutoCubeShootingCommandGroup(ArmSubsystem armSub, DrivetrainSubsystem drivetrainSub, ClawSubsystem clawSub, ElevatorSubsystem elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousArmCommand(armSub, drivetrainSub, elevatorSub, clawSub),
      new AutonomousClawOuttakeCommand(clawSub, armSub)
      //new MoveArmBackToDefaultPositionCommand(armSub)
    );
  }
}
