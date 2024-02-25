// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/* Local */
import frc.robot.Commands.LookUpShot;
import frc.robot.Commands.intakeNote;
import frc.robot.Commands.prepareToShoot;
import frc.robot.Commands.velocityOffset;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Arm.ArmDefault;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Intake.IntakeDefault;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
//import frc.robot.Util.VisionLookUpTable;
//import frc.robot.Vision.Limelight;
//import frc.robot.Vision.PhotonVision;
import frc.robot.generated.TunerConstants;
import frc.robot.Util.shootTimer;

public class RobotContainer {

    /*
     * Shuffleboard Chooser widgets
     */
    private SendableChooser<Command> autoChooser;
    private SendableChooser<String> controlChooser = new SendableChooser<>();
    private SendableChooser<Double> speedChooser = new SendableChooser<>();

    /*
     * Lookup Table
     */
    //private VisionLookUpTable m_VisionLookUpTable = new VisionLookUpTable();

    /*
     * Speed adjustments
     */
    // Initial max is true top speed
    private double m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    // Reduction in speed from Max Speed, 0.5 = 50%
    private final double m_HalfSpeed = 0.5;
    // Reduction in speed from Max Speed, 0.25 = 25%
    private final double m_QuarterSpeed = 0.25;
    // .75 rotation per second max angular velocity. Adjust for max turning rate speed.
    private final double m_MaxAngularRate = Math.PI * 1.5;
    // .75 rotation per second max angular velocity. Adjust for max turning rate speed.
    private final double m_HalfAngularRate = Math.PI * 1.0;
    // .75 rotation per second max angular velocity. Adjust for max turning rate speed.
    private final double m_QuarterAngularRate = Math.PI * 0.5;
    // Track current AngularRate
    private double m_AngularRate = m_MaxAngularRate;
    // Save last Speed Limit so we know if it needs updating
    private Double m_lastSpeed = 0.65;

    /*
     * Driver/Operator controllers
     */
    CommandXboxPS5Controller m_driverCtrl = new CommandXboxPS5Controller(0);
    CommandXboxPS5Controller m_operatorCtrl = new CommandXboxPS5Controller(1);

    // Drive Control style settings
    private Supplier<SwerveRequest> m_controlStyle;
    private String m_lastControl = "2 Joysticks";

    /*
     * Swerve Drive Configuration
     */
    // Tuner Constants is a static class that defines the drivetrain constants
    // It is configured by the Phoenix Tuner X Swerve Project Generator
    CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

    // Field-centric driving in Open Loop, can change to closed loop after characterization
    SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(m_MaxSpeed * 0.1).withRotationalDeadband(m_AngularRate * 0.1);

    // Field-centric driving in Closed Loop. Comment above and uncomment below.
    // SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
    // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(AngularRate * 0.1);

    // Swerve Drive functional requests
    SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
    SwerveRequest.FieldCentricFacingAngle m_head = new SwerveRequest.FieldCentricFacingAngle();
    SwerveRequest.FieldCentricFacingAngle m_cardinal = new SwerveRequest.FieldCentricFacingAngle();

    // Set up Drivetrain Telemetry
    Telemetry m_logger = new Telemetry(m_MaxSpeed);
    Pose2d m_odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

    // Instantiate other Subsystems
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    shootTimer m_shootTimer = new shootTimer();
    StageSubsystem m_stageSubsystem = new StageSubsystem(m_shootTimer);
    ArmSubsystem m_armSubsystem = new ArmSubsystem();
    //PhotonVision m_PhotonVision = new PhotonVision();

    // Setup Limelight periodic query (defaults to disabled)
    //Limelight m_vision = new Limelight(m_drivetrain);

    public RobotContainer() {

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Change this to specify Limelight is in use
        //m_vision.useLimelight(false);
        // m_vision.setAlliance(Alliance.Blue);
        //m_vision.trustLL(true);

        // Sets autoAim Rot PID
        m_head.HeadingController.setPID(10, 0, 0);
        //m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        

        // Sets Cardinal Rotation PID
        m_cardinal.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        m_cardinal.HeadingController.setPID(6.0, 0, 0.6);

        if (RobotConstants.kIsTuningMode) {
            SmartDashboard.putData("Auto Turning PID", m_head.HeadingController);
            SmartDashboard.putData("Cardinal Turning PID", m_cardinal.HeadingController);
        }

        // Register NamedCommands for use in PathPlanner autos
        registerNamedCommands();

        // Configure Shuffleboard Chooser widgets
        configureChooserBindings();

        // Configure Driver and Operator controller buttons
        configureButtonBindings();

        // Configure button bindings for SysID robot profiling commands
        configureSysIDProfiling();

        // Set up the Telemetry function
        m_drivetrain.registerTelemetry(m_logger::telemeterize);

        // If running in Simulation, initialize the current pose
        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        }

    }

    private void registerNamedCommands() {

        // Register Named Commands for use in PathPlanner autos
        NamedCommands.registerCommand("RunIntake", (new intakeNote(m_intakeSubsystem, m_stageSubsystem)));
        NamedCommands.registerCommand("DownIntake", m_armSubsystem.prepareForIntakeCommand());
        NamedCommands.registerCommand("StopIntake", m_intakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("RunShooter", m_shooterSubsystem.runShooterCommand(30, 35));
        NamedCommands.registerCommand("RunShooter2", m_shooterSubsystem.runShooterCommand());
        NamedCommands.registerCommand("StopShooter", m_shooterSubsystem.stopShooterCommand());
        NamedCommands.registerCommand("ShootNote", m_stageSubsystem.feedNote2ShooterCommand());
        NamedCommands.registerCommand("WingShot", new prepareToShoot(RobotConstants.WING, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        
    }

    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
     * called on robot disable to prevent any integral windup.
     */
    public void disablePIDSubsystems() {
        m_armSubsystem.disable();
    }

    private void configureChooserBindings() {

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Build a drive control style chooser
        controlChooser.setDefaultOption("2 Joysticks", "2 Joysticks");
        controlChooser.addOption("1 Joystick Rotation Triggers", "1 Joystick Rotation Triggers");
        controlChooser.addOption("Split Joysticks Rotation Triggers", "Split Joysticks Rotation Triggers");
        controlChooser.addOption("2 Joysticks with Gas Pedal", "2 Joysticks with Gas Pedal");
        SmartDashboard.putData("Control Style Chooser", controlChooser);

        // Configure a Trigger to change the Control Style when a selection is made on the Control Style Chooser
        Trigger controlPick = new Trigger(() -> m_lastControl != controlChooser.getSelected());
        controlPick.onTrue(runOnce(() -> newControlStyle()));
        
        // Set the initial Drive Control Style
        newControlStyle();

        // Build a speed limit chooser
        speedChooser.addOption("100%", 1.0);
        speedChooser.addOption("95%", 0.95);
        speedChooser.addOption("90%", 0.9);
        speedChooser.addOption("85%", 0.85);
        speedChooser.addOption("80%", 0.8);
        speedChooser.addOption("75%", 0.75);
        speedChooser.addOption("70%", 0.7);
        speedChooser.setDefaultOption("65%", 0.65);
        speedChooser.addOption("60%", 0.6);
        speedChooser.addOption("55%", 0.55);
        speedChooser.addOption("50%", 0.5);
        speedChooser.addOption("35%", 0.35);
        SmartDashboard.putData("Speed Limit", speedChooser);

        // Configure a Trigger to change the speed limit when a selection is made on the Speed Limit Chooser
        Trigger speedPick = new Trigger(() -> m_lastSpeed != speedChooser.getSelected());
        speedPick.onTrue(runOnce(() -> newSpeed()));

        // Set the initial Speed Limit
        newSpeed();

    }

    private void configureButtonBindings() {

        /*
         * Driver Controls:
         * Y Button: Rotate to North <when pressed>
         * B Button: Rotate to East <when pressed>
         * A Button: Rotate to South <when pressed>
         * X Button: Rotate to West <when pressed>
         * Start Button: Lookup shot (adjust Arm based on distance to goal)
         * DPad Left: Brake in "X" position (while held)
         * DPad Up: Reset field orientation (when pressed)
         * DPad Right:<no-op>>
         * DPad Down: <no-op>
         * Left Bumper: Reduce Speed to 50% (while held)
         * Right Bumper: Reduce Speed to 25% (while held)
         * Left Trigger: Intake Note <when pressed>
         * Right Trigger: Shoot Note <when pressed>
         * Left Stick Button: <no-op>
         * Right Stick Button: Auto Rotate to Speaker / Drive using Left Stick (while held)
         * 
         * 
         * Operator Controls:
         * Y Button: <no-op>
         * B Button: <no-op>
         * A Button: Stop Shooter 
         * X Button: Arm to STOWED Position (when pressed)
         * Start Button: <no-op>
         * DPad Left: Arm to PODIUM position & Start Shooter (when pressed)
         * DPad Up: Arm to AMP Position & Start Shooter (when pressed)
         * DPad Down: Arm to SUBWOOFER Position & Start Shooter (when pressed)
         * DPad Down: Arm to INTAKE Position (when pressed)
         * Left Bumper: Activate "Manual Arm" mode
         * Left Stick: Y-Axis will drive Arm up and down
         * Right Bumper: <no-op>
         * Left Trigger: Manual Intake (in)
         * Right Trigger: Manual Intake (out)
         * Left Stick Button: <no-op>
         * Right Stick Button: <no-op>
         * *
         */

        /*
         * DRIVER Controls
         */
        // Driver: While Y button is pressed, rotate to North
        m_driverCtrl.y().onTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(Rotation2d.fromDegrees(0.0))
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While B button is pressed, rotate to East
        m_driverCtrl.b().onTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(Rotation2d.fromDegrees(-90.0))
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While A button is pressed, rotate to South
        m_driverCtrl.a().onTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(Rotation2d.fromDegrees(180.0))
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While X button is pressed, rotate to West
        m_driverCtrl.x().onTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(Rotation2d.fromDegrees(90.0))
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While Right Stick button is pressed, drive while pointing to alliance speaker
        // AND adjusting Arm angle AND running Shooter
         m_driverCtrl.rightStick().whileTrue(Commands.parallel(
            new velocityOffset(m_drivetrain, () -> m_driverCtrl.getRightTriggerAxis()),
            m_drivetrain.applyRequest(
                () -> m_head.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed)
                        .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed)
                        .withTargetDirection(m_drivetrain.getVelocityOffset())
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)
            ),
            new LookUpShot(m_armSubsystem, m_shooterSubsystem, () -> m_drivetrain.calcDistToSpeaker())
        )); 

         // Driver: DPad Left: put swerve modules in Brake mode (modules make an 'X') (while pressed)
        m_driverCtrl.povLeft().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

         // Driver: DPad Up: Reset the field-centric heading (when pressed)
        m_driverCtrl.povUp().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        // Driver: While Left Bumper is held, reduce speed by 50%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_HalfSpeed)
                .andThen(() -> m_AngularRate = m_HalfAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed)
                .andThen(() -> m_AngularRate = m_MaxAngularRate));
        
        // Driver: While Right Bumper is held, reduce speed by 25%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_QuarterSpeed)
                .andThen(() -> m_AngularRate = m_QuarterAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed)
                .andThen(() -> m_AngularRate = m_MaxAngularRate));
        
        // Driver: When LeftTrigger is pressed, lower the Arm and then run the Intake and Stage until a Note is found
        m_driverCtrl.leftTrigger(0.4).onTrue(m_armSubsystem.prepareForIntakeCommand()
            .andThen(new intakeNote(m_intakeSubsystem, m_stageSubsystem)));

        // Driver: When RightTrigger is pressed, release Note to shooter, then lower Arm
        m_driverCtrl.rightTrigger(0.4).onTrue(m_stageSubsystem.feedNote2ShooterCommand());
            //.andThen(m_armSubsystem.prepareForIntakeCommand()));


        // Driver: While start button held, adjust Arm elevation based on goal
        //m_driverCtrl.start().onTrue(Commands.parallel(m_shooterSubsystem.runShooterCommand(),m_armSubsystem.moveToDegreeCommand()));

/*         m_driverCtrl.start().whileTrue(
            m_drivetrain.applyRequest(
                () -> m_head.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed)
                        .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed)
                        .withTargetDirection(m_drivetrain.getAngularOffset(m_stageSubsystem.getTimeAtStartOfShot()))
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)
        )); */

        /*
         * OPERATOR Controls
         */
        // Operator: When A button is pressed, stop Shooter
        //m_operatorCtrl.a().onTrue(m_shooterSubsystem.runShooterCommand());
        m_operatorCtrl.a().onFalse(m_shooterSubsystem.stopShooterCommand());

        

         // Operator: X Button: Arm to Stowed Position (when pressed)
         m_operatorCtrl.x().onTrue(new prepareToShoot(RobotConstants.STOWED, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        // Operator: Use Left Bumper and Left Stick Y-Axis to manually control Arm
        m_armSubsystem.setDefaultCommand(
            new ArmDefault(m_armSubsystem, m_operatorCtrl.leftBumper(), () -> (-1.0)*m_operatorCtrl.getLeftY())
        );

         // Operator: DPad Left: Arm to Podium position (when pressed)
         m_operatorCtrl.povLeft().onTrue(new prepareToShoot(RobotConstants.PODIUM, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Up: Shooter/Arm to AMP Position & Speed (when pressed)
        m_operatorCtrl.povUp().onTrue(new prepareToShoot(RobotConstants.AMP, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Right: Arm to Wing Position (when pressed)
         m_operatorCtrl.povRight().onTrue(new prepareToShoot(RobotConstants.WING, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Down: Arm to Subwoofer Position (when pressed)
         m_operatorCtrl.povDown().onTrue(new prepareToShoot(RobotConstants.SUBWOOFER, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        // Operator: Use Left and Right Triggers to run Intake at variable speed (left = in, right = out)
        m_intakeSubsystem.setDefaultCommand(new IntakeDefault(m_intakeSubsystem, m_stageSubsystem,
                                            ()-> m_operatorCtrl.getLeftTriggerAxis(),
                                            () -> m_operatorCtrl.getRightTriggerAxis()));

        /*
         * Put Commands on Shuffleboard
         */
        if (RobotConstants.kIsShooterTuningMode) {
            SmartDashboard.putData("Update Shooter Gains", m_shooterSubsystem.updateShooterGainsCommand());
            SmartDashboard.putData("Run Shooter", m_shooterSubsystem.runShooterCommand());
            SmartDashboard.putData("Stop Shooter", m_shooterSubsystem.stopShooterCommand());
            SmartDashboard.putData("Arm to Angle", m_armSubsystem.moveToDegreeCommand());
        }
        SmartDashboard.putData("Move Arm To Setpoint", m_armSubsystem.tuneArmSetPointCommand());
       
    }

    private void configureSysIDProfiling() {

        /*
         * These bindings will only be used when characterizing the Drivetrain. They can
         * eventually be commented out.
         */
/*
        m_driverCtrl.x().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kForward));
        m_driverCtrl.x().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kReverse));

        m_driverCtrl.y().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kForward));
        m_driverCtrl.y().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kReverse));

        m_driverCtrl.a().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kForward));
        m_driverCtrl.a().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kReverse));

        m_driverCtrl.b().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kForward));
        m_driverCtrl.b().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kReverse));

        // Drivetrain needs to be placed against a sturdy wall and test stopped
        // immediately upon wheel slip
        m_driverCtrl.back().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveSlipTest());
    */
    }

    public Command getAutonomousCommand() {

        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    private void newControlStyle() {

        m_lastControl = controlChooser.getSelected();
        switch (controlChooser.getSelected()) {
            case "2 Joysticks":
                m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Drive forward -Y
                        .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise with
                                                                                        // negative X (left)
                break;
            case "1 Joystick Rotation Triggers":
                m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Drive forward -Y
                        .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate((m_driverCtrl.getLeftTriggerAxis() - m_driverCtrl.getRightTriggerAxis())
                                * m_AngularRate);
                // Left trigger turns left, right trigger turns right
                break;
            case "Split Joysticks Rotation Triggers":
                m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed) // Left stick
                                                                                                    // forward/back
                        .withVelocityY(-m_driverCtrl.getRightX() * m_MaxSpeed) // Right stick strafe
                        .withRotationalRate((m_driverCtrl.getLeftTriggerAxis() - m_driverCtrl.getRightTriggerAxis())
                                * m_AngularRate);
                // Left trigger turns left, right trigger turns right
                break;
            case "2 Joysticks with Gas Pedal":
                m_controlStyle = () -> {
                    var stickX = -m_driverCtrl.getLeftX();
                    var stickY = -m_driverCtrl.getLeftY();
                    var angle = Math.atan2(stickX, stickY);
                    // RightTriggerAxis = "Gas pedal"
                    return m_drive.withVelocityX(Math.cos(angle) * m_driverCtrl.getRightTriggerAxis() * m_MaxSpeed) // left
                                                                                                                    // x
                                                                                                                    // *
                                                                                                                    // gas
                            .withVelocityY(Math.sin(angle) * m_driverCtrl.getRightTriggerAxis() * m_MaxSpeed) // Angle
                                                                                                              // of left
                                                                                                              // stick Y
                                                                                                              // * gas
                            .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise
                                                                                            // with negative X (left)
                };
                break;
        }
        try {
            m_drivetrain.getDefaultCommand().cancel();
        } catch (Exception e) {
        }

        // Specify the desired Control Style as the Drivetrain's default command
        // Drivetrain will execute this command periodically
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
    }

    private void newSpeed() {

        m_lastSpeed = speedChooser.getSelected();
        m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed;
    }
}
