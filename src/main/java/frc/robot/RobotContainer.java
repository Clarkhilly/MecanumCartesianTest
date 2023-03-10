// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_JoyLeft =
      new CommandJoystick(OperatorConstants.kLeftJoystickPort);
  private final CommandJoystick m_JoyRight =
      new CommandJoystick(OperatorConstants.kRightJoystickPort);

private final Joystick m_leftJoystick = new Joystick(0);
private final Joystick m_rightJoystick = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureButtonBindings();

    m_DriveSubsystem.setDefaultCommand(new MecanumDriveCmd(m_DriveSubsystem,()-> -m_JoyLeft.getY(), ()-> -m_JoyLeft.getX(), ()-> -m_JoyRight.getX()));
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
   // new Trigger(m_exampleSubsystem::exampleCondition)
       // .onTrue(new MecanumDriveCmd(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private void configureButtonBindings(){
    new JoystickButton(m_leftJoystick, 5)
      .whileTrue(new InstantCommand( ()-> m_DriveSubsystem.setMotors(1, 0, 0, 0)));


      new JoystickButton(m_leftJoystick, 3)
      .whileTrue(new InstantCommand( ()-> m_DriveSubsystem.setMotors(0, 1, 0, 0)));


      new JoystickButton(m_leftJoystick, 6)
      .whileTrue(new InstantCommand( ()-> m_DriveSubsystem.setMotors(0, 0, 1, 0)));


      new JoystickButton(m_leftJoystick, 4)
      .whileTrue(new InstantCommand( ()-> m_DriveSubsystem.setMotors(0, 0, 0, 1)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;  //Autos.exampleAuto(m_exampleSubsystem);
  }
}

