// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveBackwardPerInch extends CommandBase {
  /** Creates a new DriveForwardPerInch. */
  DrivetrainSubsystem drivetrainSub;

  double inches;
  double drivespeed;

  boolean driveForwardPerInchIsFinished;
  boolean driveForward;

  public DriveBackwardPerInch(DrivetrainSubsystem drivetrainSub, double inches, double drivespeed) { 
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.inches = inches;
    this.drivespeed = drivespeed;

    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSub.resetDrivetrainEncoders();

    System.out.println("DriveForwardPerinch Has started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrainSub.driveForward(drivespeed);
    drivetrainSub.leftMotorGroup.set(drivespeed);
    drivetrainSub.rightMotorGroup.set(drivespeed * 0.95);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSub.driveForward(0);
    System.out.println("DriveForwardPerinch Has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((drivetrainSub.averageMotorPos / Constants.DrivetrainConstants.HIGH_GEAR_ENCODER_PER_INCH) <= inches){
      return true;
    } else{
      return false;
    }
  }
}
