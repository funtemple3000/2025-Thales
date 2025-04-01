// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;



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
import frc.robot.subsystems.elevator;
import frc.robot.commands.autodispense;
import frc.robot.commands.autodispenseMax;


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

    private final CommandXboxController joystick1 = new CommandXboxController(0);
    // Driver Controller
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    // Operator Controller

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private static final String kNoAuto = "NoAuto";
    private static final String kCycleLeft = "CycleLeft";
    private static final String kCycleRight = "CycleRight";
    private static final String kCycleCenter = "CycleCenter1";
    private static final String kPracticeAuto = "Practice Auto";
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final SendableChooser<String> numCyclesChooser = new SendableChooser<>();

    public final elevator m_elevator = new elevator();
    public final feeder dispenser = new feeder(new TalonFX(40));
    public final autodispense m_autodispense = new autodispense(dispenser);
    public final autodispenseMax m_autodispenseMax = new autodispenseMax(dispenser);
    public boolean b_hold = false;

    public RobotContainer() {
        configureBindings();
        autoChooser.setDefaultOption("No Auto", kNoAuto);
        autoChooser.addOption("Cycle Left 1", kCycleLeft + "1");
        autoChooser.addOption("Cycle Left 2", kCycleLeft + "2");
        autoChooser.addOption("Cycle Left 3", kCycleLeft + "3");
        autoChooser.addOption("Cycle Right 1", kCycleRight + "1");
        autoChooser.addOption("Cycle Right 2", kCycleRight + "2");
        autoChooser.addOption("Cycle Right 3", kCycleRight + "3");
        autoChooser.addOption("Cycle Center", kCycleCenter);
        autoChooser.addOption("Practice Auto (Practice Only)", kPracticeAuto);
        SmartDashboard.putData(autoChooser);

        NamedCommands.registerCommand("autodispense", m_autodispense);
        NamedCommands.registerCommand("autodispenseMax", m_autodispenseMax);
    
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(joystick1.getLeftY(), joystick1.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.rightTrigger().onTrue(
            Commands.runOnce(()->
            dispenser.dispense())
        );

        joystick.rightTrigger().onFalse(
            Commands.runOnce(()->
            dispenser.stop())
        );
        joystick.rightBumper().onTrue(
            Commands.runOnce(()->
            dispenser.gimmemorpowa())
        );
        joystick.rightBumper().onFalse(
            Commands.runOnce(()->
            dispenser.stop())
        );
        joystick.y().onTrue(
            Commands.runOnce(()-> 
            dispenser.stop())
        );
        joystick.leftTrigger().onTrue(
            Commands.runOnce(()->
            dispenser.hold())
        );
        joystick.x().onTrue(
            Commands.runOnce(()->
            dispenser.morpowaMAX())
        );
        joystick.x().onFalse(
            Commands.runOnce(()->
            dispenser.stop())
        );

        // reset the field-centric heading on left bumper press
        joystick1.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        String autoSelected = autoChooser.getSelected();
        
        PathPlannerAuto pathAuto = new PathPlannerAuto(autoSelected);
        return pathAuto;
    }
}
