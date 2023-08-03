// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class DriveForwardToTheRampCommand extends CommandBase {
  /** Creates a new DriveForwardCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  public DriveForwardToTheRampCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Drive Forward!");
    drivetrainSub.resetDrivetrainEncoders();
    drivetrainSub.gearShiftHigh();
    drivetrainSub.resetDrivetrainEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSub.driveForward(Constants.DrivetrainConstants.DRIVE_TO_THE_RAMP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive Forward Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(navxSub.getPitch() >= 8){
      return true;
    } else{
    return false;
    }
  }
}
