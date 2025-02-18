// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;
import frc.robot.RobotState.RELATIVE_SCORE_POSITION;
import frc.robot.controls.DRIVER_CONTROLS;
import frc.robot.controls.OPERATOR_CONTROLS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawState;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraightRequest = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandSwerveDrivetrain m_swerveSubsystem = TunerConstants.createDrivetrain();

    private final CameraSubsystem m_cameraSubsystem = CameraSubsystem.getSingleton();
    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
    private final ClawSubsystem m_clawSubsystem = ClawSubsystem.getSingleton();

    private InstantCommand makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION desiredPosition, ElevatorState desiredElevatorState, ClawState desiredClawState) {
        // return new InstantCommand(() -> {
        //     RobotState.setTargetScorePosition(desiredPosition);
        //     m_elevatorSubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);
        //     m_clawSubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);

        //     m_elevatorSubsystem.setAutomaticState(desiredElevatorState);
        //     m_clawSubsystem.setAutomaticState(desiredClawState);
        // }, m_elevatorSubsystem, m_clawSubsystem);
        // TODO: put above back
        return new InstantCommand(() -> {
            RobotState.setTargetScorePosition(desiredPosition);
            m_elevatorSubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);

            m_elevatorSubsystem.setAutomaticState(desiredElevatorState);
        }, m_elevatorSubsystem);
    }
    public final InstantCommand m_positionCoralStation;
    public final InstantCommand m_setTargetScorePosition_NONE;

    public final InstantCommand m_setTargetScorePosition_L1;
    public final InstantCommand m_setTargetScorePosition_L2_L;
    public final InstantCommand m_setTargetScorePosition_L2_R;
    public final InstantCommand m_setTargetScorePosition_L3_L;
    public final InstantCommand m_setTargetScorePosition_L3_R;
    public final InstantCommand m_setTargetScorePosition_L4_L;
    public final InstantCommand m_setTargetScorePosition_L4_R;

    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_autoChooser = AutoBuilder.buildAutoChooser("thereisnoauto");
        SmartDashboard.putData("AutoChooser", m_autoChooser);

        m_positionCoralStation = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.NONE, ElevatorState.CORAL_STATION, ClawState.CORAL_STATION);
        m_setTargetScorePosition_NONE = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.NONE, ElevatorState.IDLE, ClawState.IDLE);

        m_setTargetScorePosition_L1 = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L1, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L2_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L2_L, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L2_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L2_R, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L3_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L3_L, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L3_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L3_R, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L4_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L4_L, ElevatorState.SCORE, ClawState.SCORE);
        m_setTargetScorePosition_L4_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L4_R, ElevatorState.SCORE, ClawState.SCORE);

        configureBindings();

        NamedCommands.registerCommand("ScoreL4_R", setTargetScorePosition_L4_R);
        NamedCommands.registerCommand("PositionCoralStation", positionCoralStation);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_swerveSubsystem.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_swerveSubsystem.applyRequest(() ->
                driveRequest.withVelocityX(-DRIVER_CONTROLS.driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-DRIVER_CONTROLS.driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-DRIVER_CONTROLS.driverController.getRightX() * MaxAngularRate)
            )
        );

        // joystick.a().whileTrue(swerveSubsystem.applyRequest(() -> brakeRequest));
        // joystick.b().whileTrue(swerveSubsystem.applyRequest(() ->
        //     pointRequest.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(swerveSubsystem.applyRequest(() ->
        //     forwardStraightRequest.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(swerveSubsystem.applyRequest(() ->
        //     forwardStraightRequest.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // DRIVER CONTROLS

        // reset the field-centric heading
        DRIVER_CONTROLS.seedFieldCentric.onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.seedFieldCentric()));
        DRIVER_CONTROLS.localizeToReef.whileTrue(m_cameraSubsystem.localizeToReefCommand);
        DRIVER_CONTROLS.driverController.b().onTrue(new InstantCommand(m_elevatorSubsystem::resetMotorPositions, m_elevatorSubsystem));

        // OPERATOR CONTROLS

        OPERATOR_CONTROLS.INTAKE_OUT.whileTrue(m_clawSubsystem.m_outCommand);
        OPERATOR_CONTROLS.INTAKE_IN.whileTrue(m_clawSubsystem.m_inCommand);
        OPERATOR_CONTROLS.INTAKE_SLOW.whileTrue(Commands.startEnd(() -> {
            m_clawSubsystem.setSlowMode(true);
        }, () -> {
            m_clawSubsystem.setSlowMode(false);
        }));

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(m_positionCoralStation);

        OPERATOR_CONTROLS.SCORE_L1.onTrue(m_setTargetScorePosition_L1);
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(m_setTargetScorePosition_L2_L);
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(m_setTargetScorePosition_L2_R);
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(m_setTargetScorePosition_L3_L);
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(m_setTargetScorePosition_L3_R);
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(m_setTargetScorePosition_L4_L);
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(m_setTargetScorePosition_L4_R);

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.m_manualUpCommand);
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.m_manualDownCommand);

        OPERATOR_CONTROLS.WRIST_MANUAL_OUT.whileTrue(m_clawSubsystem.m_manualWristOutCommand);
        OPERATOR_CONTROLS.WRIST_MANUAL_IN.whileTrue(m_clawSubsystem.m_manualWristInCommand);

        m_swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void teleopInit() {
        RobotState.setTargetScorePosition(RELATIVE_SCORE_POSITION.NONE);

        m_elevatorSubsystem.resetManualPosition();
        m_elevatorSubsystem.resetMotorPositions();
        m_clawSubsystem.resetManualWristPosition();
    }
}
