// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new Arm. */

  public WPI_TalonFX armMotor;
  public double armMotorPos;
  public double armDown;
  public double armUp;
  public double armError;
  public double lastTimestamp;
  public double errorSum;
  
  

  public ArmSubsystem() {
    armMotor = new WPI_TalonFX(Constants.ArmConstants.ARM_MOTOR);
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armMotorPos = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("ArmPosition", armMotorPos);
    SmartDashboard.putNumber("ArmDegrees", getArmAngle());
    SmartDashboard.putNumber("ArmError", Constants.ArmConstants.AUTONOMOUS_ARM_SETPOINT + getArmAngle());
  
  }
  
  public void armWithJoystick(double armSpeed) {
    armMotor.set(armSpeed * 0.5);
  }

  public double getArmAngle(){
    return (armMotorPos / Constants.ArmConstants.ENCODER_PER_DEGREE);
  }

  public void setArmAngleWithPID(double armAngleSetPoint){
    double armError = armAngleSetPoint - getArmAngle();
    double timeChanges = Timer.getFPGATimestamp() - lastTimestamp;
    errorSum += armError * timeChanges;
    double armMotorOutput = MathUtil.clamp(Constants.ArmConstants.ARM_kP * armError + Constants.ArmConstants.ARM_kI * errorSum, -0.4, 0.4);
    armMotor.set(armMotorOutput);
    lastTimestamp = Timer.getFPGATimestamp();
  }

  public void resetArmEncoder(){
    armMotor.setSelectedSensorPosition(0);
  }



  public void stopArmMotor() {
    armMotor.set(0);
  }
}


