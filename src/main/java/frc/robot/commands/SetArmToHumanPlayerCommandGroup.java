// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmToHumanPlayerCommandGroup extends ParallelCommandGroup {
  /** Creates a new SetArmToHumanPlayer. */
  public SetArmToHumanPlayerCommandGroup(ElevatorSubsystem elevatorSub, ArmSubsystem armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorLiftWithjoystickCommand(elevatorSub, -Constants.ElevatorConstants.QUICK_ELEVATOR_SET_HEIGHT_SPEED),
      new SetArmAngleCommand(armSub, Constants.ArmConstants.SET_ARM_FOR_HUMAN_PLAYER)
    );
  }
}
