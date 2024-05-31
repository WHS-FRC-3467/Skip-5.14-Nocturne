// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.Commands.Autos;
//import frc.robot.Commands.ExampleCommand;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain m_CommandSwerveDrivetrain = new CommandSwerveDrivetrain();
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final StageSubsystem m_StageSubsystem = new StageSubsystem();
    private final Limelight m_LimeLight = new Limelight();
    // Replace with CommandPS4Controller or CommandJoystick if needed
    CommandXboxPS5Controller m_driverController = new CommandXboxPS5Controller(0); // assuming 1 is assignment in DS
    CommandXboxPS5Controller m_operatorController = new CommandXboxPS5Controller(1);
    GenericHID m_driveRmbl = m_driverController.getHID();
    GenericHID m_operatorRmbl = m_operatorController.getHID();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        /*
         * new Trigger(m_IntakeSubsystem::exampleCondition)
         * .onTrue(new intakeFwdCommand(0.7));
         */

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        m_driverController.leftBumper().whileTrue(m_IntakeSubsystem.intakeFwdCommand(0.7));
        m_driverController.rightBumper().whileTrue(m_IntakeSubsystem.intakeRevCommand());
        // Once the button is lifted, the intake should go back to its default command
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*
     * public Command getAutonomousCommand() {
     * // An example command will be run in autonomous
     * return Autos.exampleAuto(m_exampleSubsystem);
     * }
     */
}