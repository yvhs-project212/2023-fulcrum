// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class DriveOnRampCommand extends CommandBase {
  /** Creates a new DriveOnRamp. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;


  public DriveOnRampCommand(DrivetrainSubsystem drivetrainSub,NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;

    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drive On ramp started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSub.driveForward(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSub.driveForward(0);
    System.out.println("Drive On ramp ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (navxSub.getPitch() > 5) {
      return true;
    } else {
      return false;
    }
  }
}
