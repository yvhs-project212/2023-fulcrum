// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.NavxSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.AutoCubeShootingCommandGroup;
import frc.robot.commands.ChargingStationBalancingCmdGroup;
import frc.robot.commands.ClawCloseCommand;
import frc.robot.commands.ElevatorLiftWithjoystickCommand;
import frc.robot.commands.NoAutoCommand;
import frc.robot.commands.ShootACubeThenBackupCommandGroup;
import frc.robot.commands.GearShiftHighCommand;
import frc.robot.commands.GearShiftLowCommand;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.ClawOpenCommand;
import frc.robot.commands.ClawRollersOuttakeCommand;
import frc.robot.commands.DriveOverChargeAndBalanceCmdGrp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ClawSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.ElevatorSubsystem;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed

  //Controller Files
  public static XboxController driverController = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  public static XboxController operatorController = new XboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Drivetrain Files
  private final DrivetrainSubsystem drivetrainSub = new DrivetrainSubsystem();
  private final ArcadeDriveCommand arcadeDriveComm = new ArcadeDriveCommand(drivetrainSub);
  private final GearShiftHighCommand gearShiftHighComm = new GearShiftHighCommand(drivetrainSub);
  private final GearShiftLowCommand gearShiftLowComm = new GearShiftLowCommand(drivetrainSub);

  //Gyroscope File
  private final NavxSubsystem m_NavxSubsystem = new NavxSubsystem();

  //Elevator Files
  private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
  private final ElevatorLiftWithjoystickCommand elevatorLiftComm = new ElevatorLiftWithjoystickCommand(elevatorSub, RobotContainer.operatorController.getRawAxis(Constants.OperatorConstants.OperationBinds.L_Y_AXIS));

  //Claw Files
  private final ClawSubsystem clawSub = new ClawSubsystem();
  private final ClawIntakeCommand clawIntakeComm = new ClawIntakeCommand(clawSub);
  private final ClawRollersOuttakeCommand clawRollersOuttakeComm = new ClawRollersOuttakeCommand(clawSub, Constants.ClawConstants.CLAW_REMOTE_OUTTAKE_SPEED);
  private final ClawOpenCommand clawOpenComm = new ClawOpenCommand(clawSub);
  private final ClawCloseCommand clawCloseComm = new ClawCloseCommand(clawSub);

  //Arm Files
  public static ArmSubsystem arm = new ArmSubsystem();
  private final ArmCommands armWithDPadsCmd = new ArmCommands(arm, -RobotContainer.operatorController.getRawAxis(Constants.OperatorConstants.OperationBinds.R_Y_AXIS));

  //Autonomous File
  public final Command chargingStationBalancingCmdGrp = new ChargingStationBalancingCmdGroup(drivetrainSub, m_NavxSubsystem, elevatorSub, arm);
  public final Command autoCubeShootingCmdGrp = new AutoCubeShootingCommandGroup(arm, drivetrainSub, clawSub, elevatorSub);
  public final Command noAutoComm = new NoAutoCommand(drivetrainSub);
  public final Command cubeThenBackupCmdGrp = new ShootACubeThenBackupCommandGroup(drivetrainSub, clawSub, arm, elevatorSub);
  public final Command DriveOverChargeAndBalanceCmdGrp = new DriveOverChargeAndBalanceCmdGrp(drivetrainSub, m_NavxSubsystem, arm, clawSub, elevatorSub);

  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autonomousChooser.setDefaultOption("No Autonomous", noAutoComm);
    autonomousChooser.addOption("Auto Balancing", chargingStationBalancingCmdGrp);
    autonomousChooser.addOption("Auto Cube Shooting", autoCubeShootingCmdGrp);
    autonomousChooser.addOption("Shoot A Cube Then Backup", cubeThenBackupCmdGrp);
    autonomousChooser.addOption("Score Cube And Drive Over Charge And Balance", DriveOverChargeAndBalanceCmdGrp);

    SmartDashboard.putData(autonomousChooser);

    //Settiing default commands for subsystems.
    elevatorSub.setDefaultCommand(elevatorLiftComm);  
    drivetrainSub.setDefaultCommand(arcadeDriveComm);
    arm.setDefaultCommand(armWithDPadsCmd);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //Claw Binds
    //Claw Intake
    final JoystickButton clawIntake = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    clawIntake.whileTrue(clawIntakeComm);
    //Claw Open
    final JoystickButton clawOpen = new JoystickButton(operatorController, XboxController.Button.kX.value);
    clawOpen.whileTrue(clawOpenComm);
    //Claw Close
    final JoystickButton clawClose = new JoystickButton(operatorController, XboxController.Button.kA.value);
    clawClose.whileTrue(clawCloseComm);
    //Claw Rollers Outtake
    final JoystickButton clawRollersOuttake = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    clawRollersOuttake.whileTrue(clawRollersOuttakeComm);


    //Drivetrain Binds
    //Drivetrain Gear Shift High
    final JoystickButton gearShiftHigh = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    gearShiftHigh.whileTrue(gearShiftHighComm);
    //Drivetrain Gear Shift Low
    final JoystickButton gearShiftLow = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    gearShiftLow.whileTrue(gearShiftLowComm);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 
    // An example command will be run in autonomous
    return autonomousChooser.getSelected();
  }
}