package frc.robot.Subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Gyro.Gyro;

public class Drivetrain extends SubsystemBase {
    public MecanumModule driveMotor[] = new MecanumModule[4];
    
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(Constants.SpeedLimiter);

    Gyro gyro = Gyro.getInstance();
    
    public MecanumDrivePoseEstimator PoseEstimator;
    public static Drivetrain drivetrain;

    private MecanumDrive mecanumDrive;

    private RobotConfig config;

    public Drivetrain() {
        gyro.reset();

        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {}

        for (int i = 0; i < 4; i++) driveMotor[i] = new MecanumModule(Constants.MotorID[i], i > 1);
        
        PoseEstimator = new MecanumDrivePoseEstimator(
            Constants.kinematics,
            gyro.getRotation(),
            getPosition(),
            Constants.InitialPose
        );

        mecanumDrive = new MecanumDrive(
            driveMotor[0].getMotor(),
            driveMotor[1].getMotor(),
            driveMotor[2].getMotor(),
            driveMotor[3].getMotor()
        );

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> driveChassisSpeeds(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
                return false;
            },
            this
        );
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        mecanumDrive.driveCartesian(
            xLimiter.calculate(xSpeed),
            yLimiter.calculate(ySpeed),
            zLimiter.calculate(zRotation),
            gyro.getRotation()
        );
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        mecanumDrive.driveCartesian(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            gyro.getRotation()
        );
    }

    public MecanumDriveWheelPositions getPosition() {
        return new MecanumDriveWheelPositions(
            driveMotor[0].getPosition(),
            driveMotor[2].getPosition(),
            driveMotor[1].getPosition(),
            driveMotor[3].getPosition()
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
            driveMotor[0].getVelocity(),
            driveMotor[2].getVelocity(),
            driveMotor[1].getVelocity(),
            driveMotor[3].getVelocity()
        );

        return Constants.kinematics.toChassisSpeeds(wheelSpeeds);
    }

    public void resetPose(Pose2d pose) {
        PoseEstimator.resetPosition(gyro.getRotation(), getPosition(), pose);
    }

    public Pose2d getPose() {
        return PoseEstimator.getEstimatedPosition();
    }

    public static Drivetrain getInstance() {
        if(drivetrain == null) drivetrain = new Drivetrain();
        return drivetrain;
    }
}
