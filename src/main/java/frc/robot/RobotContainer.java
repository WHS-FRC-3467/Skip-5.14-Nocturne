// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoCommands.AutoLookUpShot;
import frc.robot.AutoCommands.autoIntakeNote;
import frc.robot.AutoCommands.autoIntakeNote;
import frc.robot.AutoCommands.AutoLookUpShot;
import frc.robot.Commands.LookUpShot;
import frc.robot.Commands.autoCollectNote;
import frc.robot.Commands.calibrateLookupTable;
import frc.robot.Commands.driveToPose;
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
import frc.robot.Subsystems.LED.LEDSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Util.FieldCentricAiming;
import frc.robot.Vision.Limelight;
import frc.robot.Vision.PhotonVision;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

    /*
     * Shuffleboard Chooser widgets
     */
    private SendableChooser<Command> autoChooser;


    // Track current MaxSpeed
    private double m_MaxSpeed =  Constants.maxSpeed;;
    // Track current AngularRate
    private double m_AngularRate = Constants.maxAngularRate;
    // Save last Speed Limit so we know if it needs updating
    private Double m_lastSpeed = 1.0;

    /*
     * Driver/Operator controllers
     */
    CommandXboxPS5Controller m_driverCtrl = new CommandXboxPS5Controller(0);
    CommandXboxPS5Controller m_operatorCtrl = new CommandXboxPS5Controller(1);

    // Drive Control style settings
    private Supplier<SwerveRequest> m_controlStyle;

    /*
     * Swerve Drive Configuration
     */
    // Tuner Constants is a static class that defines the drivetrain constants
    // It is configured by the Phoenix Tuner X Swerve Project Generator
    CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

    // Field-centric driving in Open Loop, can change to closed loop after characterization
    SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(Constants.maxSpeed * 0.1).withRotationalDeadband(m_AngularRate * 0.1);
    
    // Field-centric driving in Closed Loop. Comment above and uncomment below.
    //SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
    //        .withDeadband(Constants.maxSpeed * 0.1).withRotationalDeadband(m_AngularRate * 0.1);

    // Swerve Drive functional requests
    SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.FieldCentricFacingAngle m_head = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);
            //.withSteerRequestType(SteerRequestType.MotionMagic);
    SwerveRequest.FieldCentricFacingAngle m_cardinal = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

   
    // Set up Drivetrain Telemetry
    Telemetry m_logger = new Telemetry(Constants.maxSpeed);
    Pose2d m_odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

    // Instantiate other Subsystems
    LEDSubsystem m_ledSubsystem = new LEDSubsystem();
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(m_ledSubsystem);
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    StageSubsystem m_stageSubsystem = new StageSubsystem(m_ledSubsystem);
    ArmSubsystem m_armSubsystem = new ArmSubsystem(m_ledSubsystem);
    PhotonVision m_photonVision = new PhotonVision(m_drivetrain);
    Limelight m_limelightVision = new Limelight(m_drivetrain);

    FieldCentricAiming m_fieldCentricAiming = new FieldCentricAiming();


    // Setup Limelight periodic query (defaults to disabled)
    
    public RobotContainer() {

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Change this to specify Limelight is in use
        m_limelightVision.useLimelight(true);
        // Sets forward reference for drive to always be towards red alliance
        m_drive.ForwardReference = ForwardReference.RedAlliance;
        // Creates PID for heading controller for aiming at angle
        m_head.ForwardReference = ForwardReference.RedAlliance;
        m_head.HeadingController.setP(8);
        m_head.HeadingController.setI(0);
        m_head.HeadingController.setD(0);
        m_head.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        m_head.HeadingController.setTolerance(Units.degreesToRadians(0.5));

        m_cardinal.ForwardReference = ForwardReference.RedAlliance;
        m_cardinal.HeadingController.setP(14);
        m_cardinal.HeadingController.setI(0);
        m_cardinal.HeadingController.setD(3);
        m_cardinal.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        m_cardinal.HeadingController.setTolerance(Units.degreesToRadians(0.5));

        configureSmartDashboard();

        // Register NamedCommands for use in PathPlanner autos
        registerNamedCommands();

        // Configure Shuffleboard Chooser widgets
        configureChooserBindings();

        // Configure Driver and Operator controller buttons
        configureButtonBindings();

        // Set up the Telemetry function
        m_drivetrain.registerTelemetry(m_logger::telemeterize);

        // If running in Simulation, initialize the current pose
        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        }


    }

    private void registerNamedCommands() {

        // Register Named Commands for use in PathPlanner autos
        NamedCommands.registerCommand("RunIntake", m_armSubsystem.prepareForIntakeCommand()
                .andThen(new autoIntakeNote(m_intakeSubsystem, m_stageSubsystem)));
        NamedCommands.registerCommand("DownIntake", m_armSubsystem.prepareForIntakeCommand());
        NamedCommands.registerCommand("StopIntake", m_intakeSubsystem.stopIntakeCommand());
        NamedCommands.registerCommand("RunShooter", m_shooterSubsystem.runShooterCommand(30, 35));
        NamedCommands.registerCommand("StopShooter", m_shooterSubsystem.stopShooterCommand());
        NamedCommands.registerCommand("ShootNote",
                m_stageSubsystem.feedNote2ShooterCommand());
/*         NamedCommands.registerCommand("WingShot",
                new prepareToShoot(RobotConstants.WING, () -> m_stageSubsystem.isNoteInStage(),
                        m_armSubsystem, m_shooterSubsystem)); */
         NamedCommands.registerCommand("LookUpShot",
                new AutoLookUpShot(m_armSubsystem, m_shooterSubsystem, () -> m_fieldCentricAiming.getDistToSpeaker(m_drivetrain.getState().Pose.getTranslation())));
    }

    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be
     * called on robot disable to prevent any integral windup.
     */
    public void disablePIDSubsystems() {
        m_armSubsystem.disable();
    }

    public void stopShooter() {
        m_shooterSubsystem.stopShooter();
    }

    private void configureChooserBindings() {

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Set the initial Drive Control Style
        newControlStyle();
    }

    // Inverts the joystick direction if on red alliance
    private double invertForAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return -1;
        }
        return 1;
    }
    // Adds 180 degrees if on red alliance
    private double addForAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return 180;
        }
        return 0;
    }

    private void configureSmartDashboard() {

        if (RobotConstants.kIsAutoAimTuningMode) {
            SmartDashboard.putData("Auto Turning PID", m_head.HeadingController);
        }
        if (RobotConstants.kIsShooterTuningMode) {
            SmartDashboard.putData("Update Shooter Gains", m_shooterSubsystem.updateShooterGainsCommand());
            SmartDashboard.putData("Run Shooter", m_shooterSubsystem.runShooterCommand());
            SmartDashboard.putData("Stop Shooter", m_shooterSubsystem.stopShooterCommand());
            SmartDashboard.putData("Arm to Angle", m_armSubsystem.moveToDegreeCommand());
        }
        if (RobotConstants.kIsArmTuningMode) {
            SmartDashboard.putData("Move Arm To Setpoint", m_armSubsystem.tuneArmSetPointCommand());          
        }        
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
         * Y Button: Arm to CLIMB position
         * B Button: Stop Shooter
         * A Button: Speed Up Shooter 
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

         m_driverCtrl.y().whileTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance())
                        .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance())
                        .withTargetDirection(Rotation2d.fromDegrees(0.0  + addForAlliance()))
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While B button is pressed, rotate to East
        m_driverCtrl.b().whileTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance())
                        .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance())
                        .withTargetDirection(Rotation2d.fromDegrees(-90.0 + addForAlliance()))
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While A button is pressed, rotate to South
        m_driverCtrl.a().whileTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance())
                        .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance())
                        .withTargetDirection(Rotation2d.fromDegrees(180.0  + addForAlliance()))
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While X button is pressed, rotate to West
        m_driverCtrl.x().whileTrue(m_drivetrain.applyRequest(
                () -> m_cardinal.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance())
                        .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance())
                        .withTargetDirection(Rotation2d.fromDegrees(90.0 + addForAlliance()))
                        .withDeadband(Constants.maxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)));

        // Driver: While Right Stick button is pressed, drive while pointing to alliance speaker
        // AND adjusting Arm angle AND running Shooter

        m_driverCtrl.rightStick().whileTrue(Commands.parallel(
                new velocityOffset(m_drivetrain, () -> m_driverCtrl.getRightTriggerAxis()),
                m_drivetrain.applyRequest(
                        () -> m_head.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance())
                                .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance())
                                .withTargetDirection(m_drivetrain.getVelocityOffset())
                                .withDeadband(Constants.maxSpeed * 0.1)
                                .withRotationalDeadband(0)),
                new LookUpShot(m_armSubsystem, m_shooterSubsystem, () -> m_drivetrain.getCorrectedDistance(),
                        m_ledSubsystem)));

        // Driver: DPad Left: put swerve modules in Brake mode (modules make an 'X') (while pressed)
        m_driverCtrl.povLeft().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

         // Driver: DPad Up: Reset the field-centric heading (when pressed)
        m_driverCtrl.povUp().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        // Driver: While Left Bumper is held, reduce speed by 25%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * Constants.quarterSpeed)
                .andThen(() -> m_AngularRate = Constants.quarterAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = Constants.maxSpeed)
                .andThen(() -> m_AngularRate = Constants.maxAngularRate));

        // Driver: While Right Bumper is held, reduce speed by 50%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * Constants.halfSpeed)
                .andThen(() -> m_AngularRate = Constants.halfAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = Constants.maxSpeed)
                .andThen(() -> m_AngularRate = Constants.maxAngularRate));
        
        // Driver: When LeftTrigger is pressed, lower the Arm and then run the Intake and Stage until a Note is found
        m_driverCtrl.leftTrigger(Constants.ControllerConstants.triggerThreashold).whileTrue(m_armSubsystem.prepareForIntakeCommand()
            .andThen(new intakeNote(m_intakeSubsystem, m_stageSubsystem)));

        // Driver: When RightTrigger is pressed, release Note to shooter, then lower Arm
        m_driverCtrl.rightTrigger(Constants.ControllerConstants.triggerThreashold).onTrue(m_stageSubsystem.feedNote2ShooterCommand());
            //.withTimeout(2)
            //.andThen(m_armSubsystem.prepareForIntakeCommand()));

        m_driverCtrl.start().whileTrue(new calibrateLookupTable(m_drivetrain,m_armSubsystem,m_shooterSubsystem));
/*         m_driverCtrl.start().whileTrue(m_drivetrain.applyRequest(
            () -> m_drive.withVelocityX(5)
                    .withVelocityY(0)
                    .withRotationalRate(0))); */
        //m_driverCtrl.start().whileTrue(new autoCollectNote(m_drivetrain,m_intakeSubsystem,m_stageSubsystem,m_limelightVision,m_head));
        
            
        /*
         * OPERATOR Controls
         */
        // Operator: When A button is pressed, run Shooter

         m_operatorCtrl.a().whileTrue(m_shooterSubsystem.runShooterCommand(50, 40).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
         // Operator: X Button: Arm to Stowed Position (when pressed)
         m_operatorCtrl.x().onTrue(new prepareToShoot(RobotConstants.STOWED, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        //Climb setpoint
        m_operatorCtrl.y().onTrue(new prepareToShoot(RobotConstants.CLIMB, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        m_operatorCtrl.b().onTrue(m_shooterSubsystem.stopShooterCommand());

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

        m_operatorCtrl.start().onTrue(new prepareToShoot(RobotConstants.FEED, ()->m_stageSubsystem.isNoteInStage(), m_armSubsystem, m_shooterSubsystem));

        // Operator: Use Left and Right Triggers to run Intake at variable speed (left = in, right = out)
        m_intakeSubsystem.setDefaultCommand(new IntakeDefault(m_intakeSubsystem, m_stageSubsystem,
                                            ()-> m_operatorCtrl.getLeftTriggerAxis(),
                                            () -> m_operatorCtrl.getRightTriggerAxis()));


        
       
    }


    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    private void newControlStyle() {
        m_controlStyle = () -> m_drive.withVelocityX(-m_driverCtrl.getLeftY() * Constants.maxSpeed * invertForAlliance()) // Drive forward -Y
                .withVelocityY(-m_driverCtrl.getLeftX() * Constants.maxSpeed * invertForAlliance()) // Drive left with negative X (left)
                .withRotationalRate(-m_driverCtrl.getRightX() * m_AngularRate); // Drive counterclockwise with
                                                                                // negative X (left)
        // Specify the desired Control Style as the Drivetrain's default command
        // Drivetrain will execute this command periodically
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(m_controlStyle).ignoringDisable(true));
    } 

    public void autoLEDs() {
        m_ledSubsystem.runAutonomousPatterns();
    }

    public void disabledLEDs() {
        m_ledSubsystem.runDisabledPatterns();
    }

    public void teleopLEDs() {
        m_ledSubsystem.runMatchTimerPattern();
    }

    public void teleopInitLEDs() {
        m_ledSubsystem.startTeleopPatterns();
    }
}
