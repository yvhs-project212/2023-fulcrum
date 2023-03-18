// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class DriveForwardCommand extends CommandBase {
  /** Creates a new DriveForwardCommand. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;
  ElevatorSubsystem elevatorSub;
  ArmSubsystem armSub;

  public DriveForwardCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub, ElevatorSubsystem elevatorSub, ArmSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    this.elevatorSub = elevatorSub;
    this.armSub = armSub;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Drive Forward!");
    drivetrainSub.gearShiftHigh();
    drivetrainSub.resetDrivetrainEncoders();
    elevatorSub.resetElevatorEncoder();
    armSub.resetArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSub.driveForward(Constants.DrivetrainConstants.DRIVE_FORWARD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive Forward Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(navxSub.getPitch() >= 10){
      return true;
    } else{
    return false;
    }
  }
}
