// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTurnTimedCommand extends CommandBase {
  /** Creates a new DriveTurnTimedCommand. */

  DrivetrainSubsystem drivetrainSub;

  private double duration;
  private int negative;

  private double timeSet;


  public DriveTurnTimedCommand(DrivetrainSubsystem drivetrainSub, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSub = drivetrainSub;
    this.duration = duration;

    addRequirements(drivetrainSub);

    timeSet = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveTurnTimedCmd started!");

    if (duration < 0) {
      negative = -1;
    } else {
      negative = 1;
    }

    timeSet = Math.abs(duration) + Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftMotors = negative * DrivetrainConstants.AUTO_LEFT_DRIVE_TURN_SPEED;
    double rightMotors = negative * DrivetrainConstants.AUTO_RIGHT_DRIVE_TURN_SPEED;

    drivetrainSub.setMotors(leftMotors, rightMotors);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSub.setMotors(0, 0);
    System.out.println("DriveTurnTimedCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > timeSet) {
      return true;
    } else {
      return false;
    } 
  }
}
