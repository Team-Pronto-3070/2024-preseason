package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

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
        lastAngle = new Rotation2d(chassisAngularOffset);

        driveFeedforward = new SimpleMotorFeedforward(
            Consts.Swerve.Drive.Feedforward.KS,
            Consts.Swerve.Drive.Feedforward.KV,
            Consts.Swerve.Drive.Feedforward.KA
        );
    }
}
