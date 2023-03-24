// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */

  //Initializing Motors And Solenoids
  public WPI_TalonFX clawMotor;
  public Solenoid clawSolenoid;

  public DigitalInput clawLimitSwitch;
  public boolean clawLimitEnable;
  
  public ClawSubsystem() {

    clawMotor = new WPI_TalonFX(Constants.ClawConstants.CLAW_ROLLER_MOTOR);

    clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ClawConstants.CLAW_SOLENOID);
    clawSolenoid.set(false);

    clawLimitSwitch = new DigitalInput(Constants.ClawConstants.CLAW_LIMIT_SWITCH);
 
  }


  @Override
  public void periodic() {
    if (clawLimitSwitch.get()){
      clawLimitEnable = true;
    } else{
      clawLimitEnable = false;
    }
    SmartDashboard.putBoolean("clawLimitSwitch", clawLimitEnable);
  }

  public void clawIntake(){
    if(clawLimitSwitch.get()){
        clawSolenoid.set(true);
        clawMotor.set(Constants.ClawConstants.CLAW_INTAKE_SPEED);
    } else {
        clawSolenoid.set(false);
        clawMotor.set(0);
    }
  }

  public void clawOpen(){
    clawSolenoid.set(true);
  }

  public void clawRollersOuttake(double clawOuttakeSpeed){
    clawMotor.set(clawOuttakeSpeed);
  }

  public void clawRollersIntake(){
    clawMotor.set(Constants.ClawConstants.CLAW_INTAKE_SPEED);
  }

  public void clawClose(){
    clawSolenoid.set(false);
  }

  public void clawRollersStop(){
    clawMotor.set(0);
  }

}
