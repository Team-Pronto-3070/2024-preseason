package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Consts;

public class SwerveModule {
    
    private final TalonFX driveMotor;
    private final CANSparkMax turnMotor;
    private final AbsoluteEncoder turningAbsoluteEncoder;

    private final SparkMaxPIDController turningPID;
    private final Rotation2d chassisAngularOffset;
    private final SimpleMotorFeedforward driveFeedforward;

    private Rotation2d lastAngle;

    public SwerveModule(int driveMotorID, int turnMotorID, double encoderOffset) {
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configFactoryDefault();

            // Configure PID(F) values
        driveMotor.config_kP(0, Consts.Swerve.Drive.PID.P);
        driveMotor.config_kI(0, Consts.Swerve.Drive.PID.I);
        driveMotor.config_kD(0, Consts.Swerve.Drive.PID.D);
        driveMotor.config_kF(0, Consts.Swerve.Drive.PID.F);

        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            Consts.Swerve.Drive.enableCurrentLimit,
            Consts.Swerve.Drive.continuousCurrentLimit,
            Consts.Swerve.Drive.peakCurrentLimit,
            Consts.Swerve.Drive.peakCurrentDuration
        ));

        driveMotor.configOpenloopRamp(Consts.Swerve.Drive.openLoopRamp);
        driveMotor.configClosedloopRamp(Consts.Swerve.Drive.closedLoopRamp);
        driveMotor.setInverted(Consts.Swerve.Drive.motorInvert);
        driveMotor.setNeutralMode(Consts.Swerve.Drive.neutralMode);
        driveMotor.setSelectedSensorPosition(0);

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(Consts.Swerve.Turn.idleMode);
        turnMotor.setSmartCurrentLimit(Consts.Swerve.Turn.currentLimit);

        turningAbsoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningAbsoluteEncoder.setPositionConversionFactor(Consts.Swerve.Turn.encoderPositionFactor);
        turningAbsoluteEncoder.setVelocityConversionFactor(Consts.Swerve.Turn.encoderVelocityFactor);
        turningAbsoluteEncoder.setInverted(Consts.Swerve.Turn.encoderInvert);

        turningPID = turnMotor.getPIDController();
        turningPID.setFeedbackDevice(turningAbsoluteEncoder);
        turningPID.setPositionPIDWrappingEnabled(true);
        turningPID.setPositionPIDWrappingMinInput(0);
        turningPID.setPositionPIDWrappingMaxInput(Consts.Swerve.Turn.encoderPositionFactor);
        turningPID.setP(Consts.Swerve.Turn.PID.P);
        turningPID.setI(Consts.Swerve.Turn.PID.I);
        turningPID.setD(Consts.Swerve.Turn.PID.D);
        turningPID.setFF(Consts.Swerve.Turn.PID.F);
        turningPID.setOutputRange(Consts.Swerve.Turn.PID.minOutput, Consts.Swerve.Turn.PID.maxOutput);

        turnMotor.burnFlash();

        chassisAngularOffset = Rotation2d.fromDegrees(encoderOffset);
        lastAngle = chassisAngularOffset;

        driveFeedforward = new SimpleMotorFeedforward(
            Consts.Swerve.Drive.Feedforward.KS,
            Consts.Swerve.Drive.Feedforward.KV,
            Consts.Swerve.Drive.Feedforward.KA
        );
    }

                                //TODO supposed to be `2` after?
    public void setDesiredState(SwerveModuleState2  rawDesiredState, boolean isOpenLoop) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
            new SwerveModuleState(
                rawDesiredState.speedMetersPerSecond,
                rawDesiredState.angle.plus(chassisAngularOffset)
            ),
            new Rotation2d(turningAbsoluteEncoder.getPosition())
        );

        if(isOpenLoop){
            driveMotor.set(ControlMode.PercentOutput, optimizedDesiredState.speedMetersPerSecond / Consts.Swerve.maxSpeed);
        } else {
            double wheelRPM = ((optimizedDesiredState.speedMetersPerSecond * 60) / Consts.Swerve.wheelCircumference);
            double motorRPM = wheelRPM * Consts.Swerve.gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            driveMotor.set(ControlMode.Velocity, sensorCounts,
                           DemandType.ArbitraryFeedForward, driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond));
        }

        // * Prevent rotating module if speed is less then 1% in order to prevent jittering
        Rotation2d angle = (Math.abs(optimizedDesiredState.speedMetersPerSecond) <= (Consts.Swerve.maxSpeed * 0.01)) ? 
                                lastAngle : optimizedDesiredState.angle;
        lastAngle = angle; // is it possible to forgo this line and set make the previous line an assignment to lastAngle?
        turningPID.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition,
                                0,
                                Consts.Swerve.Turn.KV * rawDesiredState.omegaRadPerSecond);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                // * the distance measured by the wheel
            driveMotor.getSelectedSensorPosition()      // get the raw position
                * Consts.Swerve.wheelCircumference      // find the number of rotations
                / (Consts.Swerve.gearRatio * 2048.0),   // correct gear ratio & units

                // * the angle of the wheel
            new Rotation2d(
                turningAbsoluteEncoder.getPosition()    // position of the motor in rotations
                - chassisAngularOffset.getRotations())
        );
    }

    public SwerveModuleState2 getState() {
        return new SwerveModuleState2(
            ((driveMotor.getSelectedSensorVelocity()    // get raw falcon units
                * (600.0 / 2048.0)                      // motor RPM
                / Consts.Swerve.gearRatio)              // wheel RPM
                * Consts.Swerve.wheelCircumference)     // wheel surface speed in m/min
                / 60.0,                                 // wheel surface speed in m/sec
            new Rotation2d(
                turningAbsoluteEncoder.getPosition() 
                - chassisAngularOffset.getRotations()),
            turningAbsoluteEncoder.getVelocity()
        );
    }
}
