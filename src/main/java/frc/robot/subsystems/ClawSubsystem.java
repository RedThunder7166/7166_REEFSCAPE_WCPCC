// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;

public class ClawSubsystem extends SubsystemBase {
    private static ClawSubsystem singleton = null;

    public static ClawSubsystem getSingleton() {
        if (singleton == null)
            singleton = new ClawSubsystem();
        return singleton;
    }

    public static enum ClawState {
        CORAL_STATION,
        IDLE,
        SCORE,
    }
    private ClawState m_state = ClawState.CORAL_STATION;
    private final StringPublisher m_statePublisher = RobotState.m_robotStateTable.getStringTopic("ClawState").publish();
    public void setAutomaticState(ClawState desiredState) {
        m_state = desiredState;
    }

    private static enum WristPosition {
        CORAL_STATION,
        IDLE,

        L1,
        L2_3,
        L4,
    }
    private WristPosition m_wristPosition = WristPosition.CORAL_STATION;
    private final StringPublisher m_wristPositionPublisher = RobotState.m_robotStateTable.getStringTopic("WristPosition").publish();
    private void setAutomaticWristPosition(WristPosition desiredPosition) {
        m_wristPosition = desiredPosition;
    }

    public static enum WristManualDirection {
        NONE,
        OUT,
        IN
    }
    private WristManualDirection m_manualWristDirection = WristManualDirection.NONE;
    private final StringPublisher m_wristManualDirectionPublisher = RobotState.m_robotStateTable.getStringTopic("WristManualDirection").publish();
    public void setManualDirection(WristManualDirection desiredManualDirection) {
        m_manualWristDirection = desiredManualDirection;
    }
    private double m_manualWristPosition = 0;
    private void setManualWristPosition(double newValue) {
        if (newValue < ClawConstants.WRIST_MIN_POSITION)
            newValue = ClawConstants.WRIST_MIN_POSITION;
        if (newValue > ClawConstants.WRIST_MAX_POSITION)
            newValue = ClawConstants.WRIST_MAX_POSITION;
        m_manualWristPosition = newValue;
    }
    public void incrementManualWristPosition(double value) {
        setManualWristPosition(m_manualWristPosition + value);
    }
    public void resetManualWristPosition() {
        setManualWristPosition(0);
    }
    private final DoublePublisher m_manualWristPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("WristManualTargetPosition").publish();

    private final TalonFX m_intakeMotor = new TalonFX(ClawConstants.INTAKE_MOTOR_ID, Constants.CANIVORE_NAME);
    private final TalonFX m_wristMotor = new TalonFX(ClawConstants.WRIST_MOTOR_ID, Constants.CANIVORE_NAME);
    private final CANcoder m_wristEncoder = new CANcoder(ClawConstants.CANCODER_ID, Constants.CANIVORE_NAME);

    private final VelocityVoltage m_intakeVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVoltage m_wristPositionControl = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage m_wristVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    private final StatusSignal<Angle> m_wristMotorPositionSignal = m_wristMotor.getPosition();
    private double m_wristMotorPositionRotations;
    private double m_wristMotorPositionDegrees;

    private final DoublePublisher m_wristMotorPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("WristMotorPosition").publish();
    private final DoublePublisher m_wristMotorPositionDegreesPublisher = RobotState.m_robotStateTable.getDoubleTopic("WristMotorPositionDegrees").publish();

    private DESIRED_CONTROL_TYPE m_desiredControlType = DESIRED_CONTROL_TYPE.AUTOMATIC;
    public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType) {
        m_desiredControlType = desiredControlType;
    }
    public DESIRED_CONTROL_TYPE getDesiredControlType() {
        return m_desiredControlType;
    }

    public ClawSubsystem() {
        // FIXME: tune intake PID
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        intakeConfig.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        intakeConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        intakeConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        intakeConfig.Slot0.kI = 0; // No output for integrated error
        intakeConfig.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        intakeConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        OurUtils.tryApplyConfig(m_intakeMotor, intakeConfig);

        // FIXME: config wrist CANcoder
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0));

        OurUtils.tryApplyConfig(m_wristEncoder, encoderConfig);

        // FIXME: tune wrist PID
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        // wristConfig.Slot0.kP = 6;
        // wristConfig.Slot0.kI = 0.18;

        wristConfig.Slot0.kG = -3.0602;
        wristConfig.Slot0.kS = 0.032;

        // wristConfig.Voltage.withPeakForwardVoltage(Volts.of(10))
        //     .withPeakReverseVoltage(Volts.of(-10));

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        wristConfig.Feedback.RotorToSensorRatio = 1; // 280
        wristConfig.Feedback.SensorToMechanismRatio = 1;

        // FIXME: tune wrist Motion Magic
        // set Motion Magic settings
        var motionMagicConfigs = wristConfig.MotionMagic;
        final double velo = 8;
        motionMagicConfigs.MotionMagicCruiseVelocity = velo; // rps
        motionMagicConfigs.MotionMagicAcceleration = velo * 2; // rps/s
        motionMagicConfigs.MotionMagicJerk = (velo * 2) * 10; // rps/s/s

        OurUtils.tryApplyConfig(m_wristMotor, wristConfig);

        OurUtils.trySetPosition(m_wristEncoder, 0);
    }

    private Command makeWristManualCommand(WristManualDirection desiredDirection) {
        return startEnd(() -> {
            setManualDirection(desiredDirection);
        }, () -> {
            setAutomaticState(ClawState.IDLE);
            setManualDirection(WristManualDirection.NONE);
        });
    }
    public final Command m_manualWristOutCommand = makeWristManualCommand(WristManualDirection.OUT);
    public final Command m_manualWristInCommand = makeWristManualCommand(WristManualDirection.IN);

    @Override
    public void periodic() {
        m_statePublisher.set(m_state.toString());

        ControlRequest intakeRequest = m_brake;
        switch (RobotState.getIntakeState()) {
            case IDLE:
                intakeRequest = m_brake; // redundant?
                break;
            case OUT:
                intakeRequest = m_intakeVelocityControl.withVelocity(ClawConstants.INTAKE_FORWARD_VELOCITY_RPS);
                break;
            case IN:
                intakeRequest = m_intakeVelocityControl.withVelocity(ClawConstants.INTAKE_BACKWARD_VELOCITY_RPS);
                break;
        }
        m_intakeMotor.setControl(intakeRequest);

        if (RobotState.ENABLE_AUTOMATIC_CLAW_CONTROL && m_manualWristDirection == WristManualDirection.NONE)
            handleWristAutomatic();
        else
            // handleWristManual();
            ;

        m_manualWristPositionPublisher.set(m_manualWristPosition);

        m_wristMotorPositionSignal.refresh();
        m_wristMotorPositionRotations = m_wristMotorPositionSignal.getValueAsDouble();
        m_wristMotorPositionDegrees = m_wristMotorPositionRotations * 360;

        m_wristMotorPositionPublisher.set(m_wristMotorPositionRotations);
        m_wristMotorPositionDegreesPublisher.set(m_wristMotorPositionDegrees);

        m_wristManualDirectionPublisher.set(m_manualWristDirection.toString());
        m_wristPositionPublisher.set(m_wristPosition.toString());
    }

    public boolean getWristHasElevatorClearance() {
        return m_wristMotorPositionDegrees >= ClawConstants.WRIST_ELEVATOR_CLEARANCE_THRESHOLD;
    }

    public void handleWristAutomatic() {
        switch (m_state) {
            case CORAL_STATION:
                setAutomaticWristPosition(WristPosition.CORAL_STATION);
                break;
            case IDLE:
                setAutomaticWristPosition(WristPosition.IDLE);
                break;
            case SCORE:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticWristPosition(WristPosition.IDLE);
                        break;

                    case L1:
                        setAutomaticWristPosition(WristPosition.L1);
                        break;

                    case L2_L:
                    case L2_R:
                    case L3_L:
                    case L3_R:
                        setAutomaticWristPosition(WristPosition.L2_3);
                        break;

                    case L4_L:
                    case L4_R:
                        setAutomaticWristPosition(WristPosition.L4);
                        break;
                }
                break;
        }

        ControlRequest wristRequest = m_brake;

        switch (m_wristPosition) {
            case CORAL_STATION:
                wristRequest = m_wristPositionControl.withPosition(ClawConstants.WRIST_CORAL_STATION_POSITION);
                break;
            case IDLE:
                wristRequest = m_brake; // redundant?
                break;

            case L1:
                wristRequest = m_wristPositionControl.withPosition(ClawConstants.WRIST_L1_POSITION);
                break;
            case L2_3:
                wristRequest = m_wristPositionControl.withPosition(ClawConstants.WRIST_L2_3_POSITION);
                break;
            case L4:
                wristRequest = m_wristPositionControl.withPosition(ClawConstants.WRIST_L4_POSITION);
                break;
        }

        m_wristMotor.setControl(wristRequest);
    }
    public void handleWristManual() {
        ControlRequest desiredControl = m_brake;

        // switch (m_manualWristDirection) {
        //     case NONE:
        //         desiredControl = m_brake; // redundant?
        //         break;
        //     case OUT:
        //         desiredControl = m_wristVelocityControl.withVelocity(ClawConstants.MANUAL_WRIST_VELOCITY_FORWARD_RPS);
        //         break;
        //     case IN:
        //         desiredControl = m_wristVelocityControl.withVelocity(ClawConstants.MANUAL_WRIST_VELOCITY_BACKWARD_RPS);
        //         break;
        // }

        final double increment = 1;

        switch (m_manualWristDirection) {
            case NONE:
                break;
            case OUT:
                incrementManualWristPosition(increment);
                break;
            case IN:
                incrementManualWristPosition(-increment);
                break;
        }

        desiredControl = m_wristPositionControl.withPosition(m_manualWristPosition / 360);

        m_wristMotor.setControl(desiredControl);
    }

    private Command makeCommand(boolean isForward) {
        return this.startEnd(() -> {
            RobotState.startIntake(isForward);
        }, () -> {
            RobotState.stopIntake();
        });
    }
    public final Command m_outCommand = makeCommand(true);
    public final Command m_inCommand = makeCommand(false);
}
