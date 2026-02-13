// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  //private final SparkMax intakeLauncherRoller;

  //private final SparkMax launchingLauncherRoller;
  //private final SparkMax launchingFeederRoller;
  private final SparkMax spinUpFeederRoller;
  private final SparkMax intakingIntakeRoller;
  private final SparkMax intakingArmRoller;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public CANFuelSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    //intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    //feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    spinUpFeederRoller = new SparkMax(SPIN_UP_FEEDER_MOTOR_ID, MotorType.kBrushless);
    intakingIntakeRoller = new SparkMax(INTAKING_INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakingArmRoller = new SparkMax(INTAKING_ARM_MOTOR_ID, MotorType.kBrushless);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking arm roller value", INTAKING_ARM_VOLTAGE); //Used to be intaking feeder value
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", FEEDER_MOTOR_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", FEEDER_MOTOR_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    //feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    //intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Create configuration for intake arm, set a current limit and apply the config to the controller
    SparkMaxConfig spinUpFeederConfig = new SparkMaxConfig();
    spinUpFeederConfig.smartCurrentLimit(SPIN_UP_FEEDER_MOTOR_CURRENT_LIMIT);
    spinUpFeederRoller.configure(spinUpFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create configuration for intake arm, set a current limit and apply the config to the controller
    SparkMaxConfig intakingIntakeConfig = new SparkMaxConfig();
    intakingIntakeConfig.smartCurrentLimit(INTAKING_INTAKE_MOTOR_CURRENT_LIMIT);
    intakingIntakeRoller.configure(intakingIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create configuration for intake arm, set a current limit and apply the config to the controller
    SparkMaxConfig intakingArmConfig = new SparkMaxConfig();
    intakingArmConfig.smartCurrentLimit(INTAKING_ARM_MOTOR_CURRENT_LIMIT);
    intakingArmRoller.configure(intakingArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    intakingIntakeRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    intakingIntakeRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Launching feeder roller value", FEEDER_MOTOR_VOLTAGE));
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", FEEDER_MOTOR_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    //feederRoller.set(0);
    //intakeLauncherRoller.set(0);

    feederRoller.set(0);
    spinUpFeederRoller.set(0);
    intakingIntakeRoller.set(0);
    intakingArmRoller.set(0);
  }

  public void armRelease() {
  intakingArmRoller
      .setVoltage(SmartDashboard.getNumber("Intaking arm roller value", INTAKING_ARM_MOTOR_ID));
  }


  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  // constants used if nothing inside smart dashboard
  public void spinUp() {
    spinUpFeederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", FEEDER_MOTOR_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command launchCommand() {
    return this.run(() -> launch());
  }
  
  public Command ejectCommand() {
    return this.run(() -> eject());
  }

  public Command stopCommand() {
    return this.run(() -> stop());
  }

  public Command armReleaseCommand() {
    return this.run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Increases voltage of motors for shooter to shoot balls higher and faster 
  public void speedIncrease()
  {
    if (FEEDER_MOTOR_VOLTAGE < 15.0)
    {
      FEEDER_MOTOR_VOLTAGE += 1.0;
      SPIN_UP_FEEDER_VOLTAGE -= 1.0;
      System.out.println(FEEDER_MOTOR_VOLTAGE);
    }
  }
  
  // Decreases voltage of motors for shooter to shoot balls lower and slower 
  public void speedDecrease()
  {
    if (FEEDER_MOTOR_VOLTAGE > 5.0)
    {
      FEEDER_MOTOR_VOLTAGE -= 1.0;
      FEEDER_MOTOR_VOLTAGE -= 1.0;
      SPIN_UP_FEEDER_VOLTAGE += 1.0;
      System.out.println(FEEDER_MOTOR_VOLTAGE);
    }
  }

}
