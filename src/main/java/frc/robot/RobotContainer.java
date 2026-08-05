// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Hello Future readers, So, you just probably used Tuner X to make your project
// which in that case, nice job! You have a working DT! The next thing that
// you have to do, is, frankly, a lot of things. The first thing that I care about
// is getting the basic visionless code in. So, id say, look at what your code and this code differs
// and copy it down.
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Swerve Requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    /* Telemetry and Dashboard Widgets */
    public final Field2d m_field = new Field2d();
    private SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> mirrorChooser = new SendableChooser<>();

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // Publish field mapping element to Glass
        SmartDashboard.putData("Field", m_field);

        // Configure manual mirror toggle options
        mirrorChooser.setDefaultOption("Don't Mirror (Blue)", false);
        mirrorChooser.addOption("Mirror Path (Red)", true);
        SmartDashboard.putData("Manual Mirror Override", mirrorChooser);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder with the custom Glass toggle option
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                drivetrain::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> {
                    // Check our custom Glass dropdown preference instead of the FMS network alliance
                    return mirrorChooser.getSelected() != null ? mirrorChooser.getSelected() : false;
                },
                drivetrain
            );

            // Populate auto options safely inside initialization block
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Mode", autoChooser);

        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner AutoBuilder!", e.getStackTrace());
            e.printStackTrace();
        }

        configureBindings();
    }

    /** Helper method used by AutoBuilder to drive robot-relative coordinates */
    private void driveRobotRelative(ChassisSpeeds speeds) {
        drivetrain.setControl(robotCentricDrive.withVelocityX(speeds.vxMetersPerSecond)
                                               .withVelocityY(speeds.vyMetersPerSecond)
                                               .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                               .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                               .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> 
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId Logs Configuration
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Returns the autonomous selection made on the driver dashboard menu */
    public Command getAutonomousCommand() {
        return autoChooser != null ? autoChooser.getSelected() : null;
    }

    /** Returns the active Field2d tracker instance to update simulated odometry points */
    public Field2d SmartDashboardFieldInstance() {
        return m_field;
    }
}
