import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;

    private final ADIS16448_IMU gyro;
    private double gyroOffset = 0.0;

    public final SwerveDriveKinematics2 kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(
                Constants.Swerve.FrontLeft.driveID,
                Constants.Swerve.FrontLeft.turnID,
                Constants.Swerve.FrontLeft.offset);

        frontRight = new SwerveModule(
                Constants.Swerve.FrontRight.driveID,
                Constants.Swerve.FrontRight.turnID,
                Constants.Swerve.FrontRight.offset);

        rearLeft = new SwerveModule(
                Constants.Swerve.RearLeft.driveID,
                Constants.Swerve.RearLeft.turnID,
                Constants.Swerve.RearLeft.offset);

        rearRight = new SwerveModule(
                Constants.Swerve.RearRight.driveID,
                Constants.Swerve.RearRight.turnID,
                Constants.Swerve.RearRight.offset);

        gyro = new ADIS16448_IMU();

        kinematics = new SwerveDriveKinematics2(
                new Translation2d(Constants.Swerve.wheelBase / 2, Constants.Swerve.trackWidth / 2),
                new Translation2d(Constants.Swerve.wheelBase / 2, -Constants.Swerve.trackWidth / 2),
                new Translation2d(-Constants.Swerve.wheelBase / 2, Constants.Swerve.trackWidth / 2),
                new Translation2d(-Constants.Swerve.wheelBase / 2, -Constants.Swerve.trackWidth / 2));

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        rearLeft.getPosition(),
                        rearRight.getPosition()
                }, new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("field", field);

    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle() - gyroOffset);
    }

    public double getPitch() {
        return gyro.getYComplementaryAngle();
        // return gyro.getAngle(gyro.getPitchAxis());
        // ! return gyro.getAngle(gyro.getRollAxis()); this code was used in the 2024
        // robot, but the function no longer exists
    }

    public void resetGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState());
    }

}