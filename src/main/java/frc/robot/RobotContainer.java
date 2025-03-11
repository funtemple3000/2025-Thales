// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private static final String kNoAuto = "NoAuto";
    private static final String kCycleLeft = "CycleLeft";
    private static final String kCycleRight = "CycleRight";
    private static final String kCycleCenter = "CycleCenter";
    private static final String kPracticeAuto = "PracticeAuto";
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final SendableChooser<String> numCyclesChooser = new SendableChooser<>();

    public final feeder dispenser = new feeder(new TalonFX(51));

    public RobotContainer() {
        configureBindings();
        autoChooser.setDefaultOption("No Auto", kNoAuto);
        autoChooser.addOption("Cycle Left", kCycleLeft);
        autoChooser.addOption("Cycle Right", kCycleRight);
        autoChooser.addOption("Cycle Center", kCycleCenter);
        autoChooser.addOption("Practice Auto (Practice Only)", kPracticeAuto);
        SmartDashboard.putData(autoChooser);

        numCyclesChooser.setDefaultOption("One Cycle", "1");
        numCyclesChooser.addOption("Two Cycles", "2");
        numCyclesChooser.addOption("Three Cycles", "3");
        SmartDashboard.putData(numCyclesChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(joystick.getLeftY(), joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.rightBumper().onTrue(
            Commands.runOnce(()->
            dispenser.dispense())
        );

        joystick.rightBumper().onFalse(
            Commands.runOnce(()->
            dispenser.stop())
        );
        joystick.x().onTrue(
            Commands.runOnce(()->
            dispenser.gimmemorpowa())
        );

        joystick.x().onFalse(
            Commands.runOnce(()->
            dispenser.stop())
        );

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        String autoSelected = autoChooser.getSelected();
        String cyclesSelected = numCyclesChooser.getSelected();

        // Note: Cycle Center only has 1 cycle.
        if (autoSelected.equals(kCycleCenter)){
            cyclesSelected = "1";
        }
        if (autoSelected.equals(kPracticeAuto)){
            cyclesSelected = ""; // Practice auto does not have "cycles"
        }
        PathPlannerAuto pathAuto = new PathPlannerAuto(autoSelected + cyclesSelected);
        return pathAuto;
    }
}
