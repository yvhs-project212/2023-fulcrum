// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NavxSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOverChargeAndBalanceCmdGrp extends SequentialCommandGroup {
  /** Creates a new TurnAndDrive. */
  public DriveOverChargeAndBalanceCmdGrp(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub, ArmSubsystem armSub, ClawSubsystem clawSub, ElevatorSubsystem elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoCubeShootingCommandGroup(armSub, drivetrainSub, clawSub, elevatorSub),
      new DriveUpRampBackwardsCommand(drivetrainSub, navxSub),
      new DriveOnRampCommand(drivetrainSub, navxSub),
      new DriveForwardPerInch(drivetrainSub, -30, -0.4),
      new DriveForwardToTheRampCommand(drivetrainSub, navxSub),
      new DriveUpTheRampCommand(drivetrainSub, navxSub)
    );
  }
}
