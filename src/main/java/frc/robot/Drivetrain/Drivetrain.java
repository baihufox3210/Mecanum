package frc.robot.Drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Drivetrain {
    public SparkMax LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor;
    public RelativeEncoder LeftFrontEncoder, LeftBackEncoder, RightFrontEncoder, RightBackEncoder;
    public SparkMaxConfig LeftFrontConfig, LeftBackConfig, RightFrontConfig, RightBackConfig;
    
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(Constants.SpeedLimiter);

    AHRS gyro;

    public MecanumDrivePoseEstimator PoseEstimator;
    public static Drivetrain drivetrain;

    private MecanumDrive mecanumDrive;

    private Drivetrain() {
        LeftFrontMotor = new SparkMax(Constants.LeftFrontMotorID, MotorType.kBrushless);
        LeftBackMotor = new SparkMax(Constants.LeftBackMotorID, MotorType.kBrushless);
        RightFrontMotor = new SparkMax(Constants.RightFrontMotorID, MotorType.kBrushless);
        RightBackMotor = new SparkMax(Constants.RightBackMotorID, MotorType.kBrushless);

        LeftFrontEncoder = LeftFrontMotor.getEncoder();
        LeftBackEncoder = LeftBackMotor.getEncoder();
        RightFrontEncoder = RightFrontMotor.getEncoder();
        RightBackEncoder = RightBackMotor.getEncoder();

        gyro = new AHRS(NavXComType.kMXP_SPI);

        PoseEstimator = 
            new MecanumDrivePoseEstimator(
                Constants.kinematics,
                gyro.getRotation2d(),
                getPosition(),
                Constants.InitialPose
            );

        LeftFrontConfig = new SparkMaxConfig();
        LeftBackConfig = new SparkMaxConfig();
        RightFrontConfig = new SparkMaxConfig();
        RightBackConfig = new SparkMaxConfig();

        LeftFrontConfig.idleMode(Constants.MotorMode)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        LeftFrontConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        LeftBackConfig.idleMode(Constants.MotorMode)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        LeftBackConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        RightFrontConfig.idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(44);
        
        RightFrontConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        RightBackConfig.idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        RightBackConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        LeftFrontMotor.configure(
            LeftFrontConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        LeftBackMotor.configure(
            LeftBackConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        RightFrontMotor.configure(
            RightFrontConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        RightBackMotor.configure(
            RightBackConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        mecanumDrive = new MecanumDrive(
            LeftFrontMotor,
            LeftBackMotor,
            RightFrontMotor,
            RightBackMotor
        );
    }

    private double getCurrentX() {
        return PoseEstimator.getEstimatedPosition().getX();
    }

    private double getCurrentY() {
        return PoseEstimator.getEstimatedPosition().getY();
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        mecanumDrive.driveCartesian(
            xLimiter.calculate(xSpeed),
            yLimiter.calculate(ySpeed),
            zLimiter.calculate(zRotation),
            gyro.getRotation2d()
        );
    }

    public void drivePID(double targetX, double targetY, double targetTheta) {
        double vx = Constants.xPID.calculate(getCurrentX(), targetX);
        double vy = Constants.yPID.calculate(getCurrentY(), targetY);
        double omega = Constants.zPID.calculate(gyro.getRotation2d().getDegrees(), targetTheta);
        drive(vx, vy, omega);
    }

    public void turnAngle(double angle) {
        double currentAngle = gyro.getRotation2d().getDegrees();
        drive(0.0, 0.0, Constants.turnPID.calculate(currentAngle, angle));
    }

    public MecanumDriveWheelPositions getPosition() {
        return new MecanumDriveWheelPositions(
            LeftFrontEncoder.getPosition(),
            LeftBackEncoder.getPosition(),
            RightFrontEncoder.getPosition(),
            RightBackEncoder.getPosition()
        );
    }

    public static Drivetrain getInstance() {
        if(drivetrain == null) drivetrain = new Drivetrain();
        return drivetrain;
    }
}
