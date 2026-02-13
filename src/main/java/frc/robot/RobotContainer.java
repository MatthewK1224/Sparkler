// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    
  // The robot's subsystems
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_robotDrive.setDefaultCommand(
        // the left stick controls translation of the robot.
        // turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    autoChooser.setDefaultOption("Default Auto", new PathPlannerAuto("DefaultAuto"));
    autoChooser.addOption("My Auto", new PathPlannerAuto("MyAuto"));

    NamedCommands.registerCommand("intake", ballSubsystem.intakeCommand());
    NamedCommands.registerCommand("eject", ballSubsystem.ejectCommand());
    NamedCommands.registerCommand("launch", ballSubsystem.launchCommand());
    NamedCommands.registerCommand("stop", ballSubsystem.stopCommand());
    NamedCommands.registerCommand("spinUp", ballSubsystem.spinUpCommand());
    NamedCommands.registerCommand("armRelease", ballSubsystem.armReleaseCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* BUMPERS AND TRIGGERS*/
    operatorController.leftBumper()
        .onTrue(new InstantCommand(() -> ballSubsystem.speedDecrease(), ballSubsystem));
    
    operatorController.rightBumper()
        .onTrue(new InstantCommand(() -> ballSubsystem.speedIncrease(), ballSubsystem));

    // While the left trigger on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop (shooter).
    operatorController.leftTrigger()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));
    operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    
    /* BUTTONS */
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    // While the Y button on operator controller is held, intake Fuel
    operatorController.y()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));

    driverController.leftBumper()
        .onTrue(new InstantCommand(() -> m_robotDrive.speedDecrease(), m_robotDrive));
    
    driverController.rightBumper()
        .onTrue(new InstantCommand(() -> m_robotDrive.speedIncrease(), m_robotDrive));

    driverController.b()
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new CommandXboxController(0).leftTrigger(.2).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
