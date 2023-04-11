// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class AutonomousClawOuttakeCommand extends CommandBase {
  /** Creates a new AutonomousClawOuttakeCommand. */

  ClawSubsystem clawSub;
  ArmSubsystem armSub;
  //int time;

  public AutonomousClawOuttakeCommand(ClawSubsystem clawSub, ArmSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.clawSub = clawSub;
    this.armSub = armSub;
    addRequirements(clawSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSub.resetClawEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*for (time = 0; time <= Constants.ClawConstants.AUTO_CLAW_OUTTAKE_TIMING; time++){
      if(time < Constants.ClawConstants.AUTO_CLAW_OUTTAKE_TIMING){
        clawSub.clawRollersOuttake(Constants.ClawConstants.CLAW_AUTO_OUTTAKE_SPEED);
      } else{
        clawSub.clawRollersStop();
      }
    }*/

    clawSub.clawRollersOuttake(Constants.ClawConstants.CLAW_AUTO_OUTTAKE_SPEED);

    //SmartDashboard.putNumber("TIme", time);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSub.clawRollersStop();
    System.out.println("Claw Shooting Finished");
    //armSub.setArmAngleWithPID(-20);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(clawSub.clawEncoderValue >= Constants.ClawConstants.AUTO_CLAW_ENCODER_SETPOINT){
      return true;
    } else{
      return false;
    }
  }
}
