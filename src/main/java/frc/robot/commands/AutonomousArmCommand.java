// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousArmCommand extends CommandBase {
  /** Creates a new AutonomousArmCommand. */

  ArmSubsystem armSub;
  DrivetrainSubsystem drivetrainSub;

  public AutonomousArmCommand(ArmSubsystem armSub, DrivetrainSubsystem drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSub = armSub;
    this.drivetrainSub = drivetrainSub;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSub.gearShiftLow();
    armSub.resetArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setArmAngleWithPID(Constants.ArmConstants.AUTONOMOUS_ARM_SETPOINT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.armMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armSub.armError <= 5){
      return true;
    } else{
      return false;
    }
  }
}
