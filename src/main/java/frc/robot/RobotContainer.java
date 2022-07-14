// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// link to original: https://github.com/QuackingBob/Swerve-Drive-Simulation.git 

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonPathExample;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

  private final XboxController driveJoystick = new XboxController(Constants.SwerveDrivetrain.kDriveJoystickPort);

  private final SendableChooser<Command> autonSelector = new SendableChooser<>();
  
  private final LimeLight limelight = new LimeLight();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // joystick drive config
    swerveDrivetrain.setDefaultCommand(new SwerveJoystickCommand(
      swerveDrivetrain, 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveXAxis), 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveYAxis), 
      () -> -driveJoystick.getRawAxis(Constants.SwerveDrivetrain.kDriveWAxis), 
      () -> driveJoystick.getRawButton(Constants.SwerveDrivetrain.kDriveFieldOrientButtonIdx)));

    //Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // button to zero the heading
    new JoystickButton(driveJoystick, 2).whenPressed(() -> swerveDrivetrain.zeroHeading());
    // buttons to change the rotation point for evasive maneuvers 
    new JoystickButton(driveJoystick, 5)
      .whenPressed(() -> swerveDrivetrain.setRotationPointIdx(1))
      .whenReleased(() -> swerveDrivetrain.setRotationPointIdx(0));
    new JoystickButton(driveJoystick, 6)
      .whenPressed(() -> swerveDrivetrain.setRotationPointIdx(2))
      .whenReleased(() -> swerveDrivetrain.setRotationPointIdx(0));
    new JoystickButton(driveJoystick, 7)
      .whenPressed(() -> swerveDrivetrain.setRotationPointIdx(3))
      .whenReleased(() -> swerveDrivetrain.setRotationPointIdx(0));
    new JoystickButton(driveJoystick, 8)
      .whenPressed(() -> swerveDrivetrain.setRotationPointIdx(4))
      .whenReleased(() -> swerveDrivetrain.setRotationPointIdx(0));
    
    autonSelector.setDefaultOption("Example", new AutonPathExample(swerveDrivetrain));
    SmartDashboard.putData("Auton Selector", autonSelector);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonSelector.getSelected();
  }
}
