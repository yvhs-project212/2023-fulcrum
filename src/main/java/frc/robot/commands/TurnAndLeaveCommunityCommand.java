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
    drivetrainSub.turnAndLeaveCommunity = false;
    drivetrainSub.turnAndLeaveCommunityFinished = false;
    drivetrainSub.recentPosition = drivetrainSub.leftTopMotorPos + 30000;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (drivetrainSub.turnAndLeaveCommunity == false) {
      if (drivetrainSub.leftTopMotorPos < drivetrainSub.recentPosition) {
        drivetrainSub.driveForward(-0.2);
        System.out.println("Going backwards");
      } else {
        drivetrainSub.driveForward(0);
        System.out.println("Going backwards stopped");
        drivetrainSub.turnAndLeaveCommunity = true;
      }
    } else if (drivetrainSub.turnAndLeaveCommunity == true) {
      if (navxSub.getYaw() < 150) {
        drivetrainSub.turnRobotRight(0.1, 0.09);
        System.out.println("truning ");
      } else {
        drivetrainSub.turnRobotRight(0, 0);
        System.out.println("turning Stopped");
        drivetrainSub.turnAndLeaveCommunityFinished = true;
      }
    }

    /*if (drivetrainSub.turnAndLeaveCommunity == false  && drivetrainSub.leftTopMotorPos < drivetrainSub.recentPosition) {
      drivetrainSub.driveForward(-0.2);
      System.out.println("Going backwards");
    } else if (drivetrainSub.turnAndLeaveCommunity == false  && drivetrainSub.leftTopMotorPos >= drivetrainSub.recentPosition) {
      drivetrainSub.driveForward(0);
      System.out.println("Going backwards stopped");
      drivetrainSub.turnAndLeaveCommunity = true;
    } else if (navxSub.getYaw() < 150 && drivetrainSub.turnAndLeaveCommunity == true) {
      drivetrainSub.turnRobotRight(0.1, 0.09);
      System.out.println("truning ");
    } else {
      drivetrainSub.turnRobotRight(0, 0);
      System.out.println("turning Stopped");
      drivetrainSub.turnAndLeaveCommunityFinished = true;
    }*/
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
