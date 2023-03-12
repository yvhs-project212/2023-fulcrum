// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.Constants;

public class OutOfCommunityCommand extends CommandBase {
  /** Creates a new OutOfCommunityCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  public OutOfCommunityCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("out of community command started");
    drivetrainSub.onChargeStation = false;
    drivetrainSub.outOfCommunity = false;
    drivetrainSub.startingYaw = navxSub.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (drivetrainSub.outOfCommunity == false) {
      if (navxSub.getYaw() < (drivetrainSub.startingYaw + 15.0)) {
        drivetrainSub.turnRobotRight(Constants.DrivetrainConstants.DRIVE_BACKWARDS_SPEED, Constants.DrivetrainConstants.DRIVE_FORWARD_SPEED);
      } else if (navxSub.getYaw() >= (drivetrainSub.startingYaw + 15.0)) {
        drivetrainSub.turnRobotRight(0, 0);
        drivetrainSub.outOfCommunity = true;
      }
    } else {
      System.out.println("Not turning 15 degrees using nav x or turning 15 degrees is over");
    }

    if (drivetrainSub.outOfCommunity == true) {
      if (drivetrainSub.onChargeStation == false && navxSub.getPitch() > -5) {
        drivetrainSub.driveBackwards(Constants.DrivetrainConstants.DRIVE_BACKWARDS_SPEED);
      } else if (navxSub.getPitch() <-5) {
        drivetrainSub.driveBackwards(Constants.DrivetrainConstants.DRIVE_SLOWLY_SPEED);
        drivetrainSub.onChargeStation = true;
        drivetrainSub.recentPosition = drivetrainSub.leftBottomMotorPos;
      } else if (drivetrainSub.onChargeStation == true && navxSub.getPitch() > -5) {
        if (drivetrainSub.leftBottomMotorPos < drivetrainSub.recentPosition) {
          drivetrainSub.driveBackwards(Constants.DrivetrainConstants.DRIVE_BACKWARDS_SPEED);
        } else if (drivetrainSub.leftBottomMotorPos >= (drivetrainSub.recentPosition + 10951.4)) {
          drivetrainSub.driveBackwards(0);
          drivetrainSub.outOfCommunity = true;
          drivetrainSub.outOfCommunityFinished = true;
        }
      }
    } else {
      System.out.println(" not running out of community code");
      drivetrainSub.driveBackwards(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Out of community ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrainSub.outOfCommunityFinished == true) {
      return true;
    } else {
      return false;
    }
  }
}
