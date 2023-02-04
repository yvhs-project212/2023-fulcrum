// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

public class NavxSubsystem extends SubsystemBase {
  /** Creates a new Navx. */
  
  private AHRS gyroScope;
  private double yaw;
  private double pitch;
  private double roll;
  private double worldLinearAccelX;
  private double worldLinearAccelY;
  private double worldLinearAccelZ;
  // Creates double methods and a gyroscope method 
   
  public NavxSubsystem() {
    gyroScope = new AHRS(SPI.Port.kMXP);
    // Configures "gyroscope" to the nav x port
  }

  @Override
  public void periodic() {
    worldLinearAccelX = gyroScope.getWorldLinearAccelX();
    worldLinearAccelY = gyroScope.getWorldLinearAccelY();
    worldLinearAccelZ = gyroScope.getWorldLinearAccelZ();
    yaw = gyroScope.getYaw();
    pitch = gyroScope.getPitch();
    roll = gyroScope.getRoll();
    // Sets double methods to the gyroscope values

    SmartDashboard.putNumber("YawValue", yaw);
    SmartDashboard.putNumber("PitchValue", pitch);
    SmartDashboard.putNumber("RollValue", roll);
    // Displays yaw, pitch, and roll values onto smartdashboard

    SmartDashboard.putNumber("getWorldLinearAccelX", Math.floor(100*worldLinearAccelX+0.5)/100.0);
    SmartDashboard.putNumber("getWorldLinearAccelY", Math.floor(100*worldLinearAccelY+0.5)/100.0);
    SmartDashboard.putNumber("getWorldLinearAccelZ", Math.floor(100*worldLinearAccelZ+0.5)/100.0);
    // Displays accel values of X, Y, and Z and rounds it up to the nearest hundredth
          
    // This method will be called once per scheduler run
  }
}
