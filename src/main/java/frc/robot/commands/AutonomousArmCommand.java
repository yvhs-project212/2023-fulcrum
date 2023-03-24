// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutonomousArmCommand extends CommandBase {
  /** Creates a new AutonomousArmCommand. */

  ArmSubsystem armSub;
  DrivetrainSubsystem drivetrainSub;
  ElevatorSubsystem elevatorSub;

  public AutonomousArmCommand(ArmSubsystem armSub, DrivetrainSubsystem drivetrainSub, ElevatorSubsystem elevatorSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSub = armSub;
    this.drivetrainSub = drivetrainSub;
    this.elevatorSub = elevatorSub;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainSub.gearShiftHigh();
    armSub.resetArmEncoder();
    elevatorSub.resetElevatorEncoder();
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
    if(armSub.positiveArmError <= 5){
      return true;
    } else{
      return false;
    }
  }
}
