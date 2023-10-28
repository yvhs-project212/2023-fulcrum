// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommands extends CommandBase {
  /** Creates a new ArmCommands. */
    public ArmSubsystem arm;
    double armSpeed;

    public ArmCommands(ArmSubsystem arm, double armSpeed) {
      this.arm = arm;
      this.armSpeed = armSpeed;
      addRequirements(arm);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmWithDPadsCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1)) {
      if (DriverStation.getJoystickIsXbox(1) && DriverStation.getJoystickIsXbox(0)) {
        arm.armWithJoystick(-RobotContainer.operatorController.getRawAxis(Constants.OperatorConstants.OperationBinds.R_Y_AXIS));
      } else if (DriverStation.getJoystickIsXbox(0) && DriverStation.getJoystickName(1) == "Generic USB Joystick") {
        arm.armWithJoystickGP(RobotContainer.gamepadController.getRawButton(7), RobotContainer.gamepadController.getRawButton(8));
      }
    } else if (DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1) == false) {
      if (DriverStation.getJoystickIsXbox(0)){
        arm.armWithJoystick(-RobotContainer.operatorController.getRawAxis(Constants.OperatorConstants.OperationBinds.R_Y_AXIS));
      } else if (DriverStation.getJoystickName(0) == "Generic USB Joystick"){
        arm.armWithJoystickGP(RobotContainer.gamepadController.getRawButton(7), RobotContainer.gamepadController.getRawButton(8));
      }
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ArmWithDPadsCmd ended!");}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
