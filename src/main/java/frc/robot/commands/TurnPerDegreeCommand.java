// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavxSubsystem;

public class TurnPerDegreeCommand extends CommandBase {
  /** Creates a new TurnPerDegree. */

  DrivetrainSubsystem drivetrainSub;
  NavxSubsystem navxSub;

  double degreeInput;

  public TurnPerDegreeCommand(DrivetrainSubsystem drivetrainSub, NavxSubsystem navxSub, double degreeInput) {
    //when method is called it will require the subsytem it uses the gyroscope and the amount of degrees it uses in the perameters

    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrainSub = drivetrainSub;
    this.navxSub = navxSub;
    this.degreeInput = degreeInput;

    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    navxSub.resetGyro();

    System.out.println("Turn per degree started");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrainSub.onHighGear == true){
     drivetrainSub.turnRobotRight(0.45, -0.45);
    } else if (drivetrainSub.onHighGear == false) {
      drivetrainSub.turnRobotRight(0.3, -0.3);
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSub.turnRobotRight(0, 0);
    System.out.println("Turn per degree ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (navxSub.getYaw() > degreeInput) {
      return true;
    } else {
      return false;
    }
  }
} 
