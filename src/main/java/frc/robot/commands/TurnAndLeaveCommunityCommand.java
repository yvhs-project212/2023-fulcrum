// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;
import frc.robot.Constants;

public class TurnAndLeaveCommunityCommand extends CommandBase {
  /** Creates a new TurnAndLeaveCommunityCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  public TurnAndLeaveCommunityCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("out of community command started");
    navxSub.resetGyro();
    drivetrainSub.turnAndLeaveCommunity = true;
    drivetrainSub.recentPosition = drivetrainSub.leftTopMotorPos + 10000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    
    if (navxSub.getYaw() < 180 && drivetrainSub.turnAndLeaveCommunity == false) {
      drivetrainSub.turnRobotRight(Constants.DrivetrainConstants.DRIVE_BACKWARDS_SPEED, Constants.DrivetrainConstants.DRIVE_FORWARD_SPEED);
      System.out.println("turning");
    } else if (navxSub.getYaw() >= 180 && drivetrainSub.turnAndLeaveCommunity == false) {
      drivetrainSub.turnRobotRight(0, 0);
      System.out.println("truning stopped");
      drivetrainSub.recentPosition = drivetrainSub.leftTopMotorPos;
      drivetrainSub.turnAndLeaveCommunity = true;
    }

    if (drivetrainSub.turnAndLeaveCommunity == true && drivetrainSub.leftTopMotorPos < drivetrainSub.recentPosition) {
      drivetrainSub.driveForward(0.4);
      System.out.println("Going forward");
    } else if (drivetrainSub.turnAndLeaveCommunity == true && drivetrainSub.leftTopMotorPos >= drivetrainSub.recentPosition) {
      drivetrainSub.driveForward(0);
      drivetrainSub.turnAndLeaveCommunityFinished = true;
      System.out.println("Going forward stopped");
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turn and leave community is finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrainSub.turnAndLeaveCommunityFinished == true) {
      return true;
    } else {
      return false;
    }
  }
}
