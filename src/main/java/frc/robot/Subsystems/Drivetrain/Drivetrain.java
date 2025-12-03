package frc.robot.Subsystems.Drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Drivetrain {
    public MecanumModule driveMotor[] = new MecanumModule[4];
    
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.SpeedLimiter);
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(Constants.SpeedLimiter);

    AHRS gyro;

    public MecanumDrivePoseEstimator PoseEstimator;
    public static Drivetrain drivetrain;

    private MecanumDrive mecanumDrive;

    private Drivetrain() {
        for (int i = 0; i < 4; i++) driveMotor[i] = new MecanumModule(Constants.MotorID[i], i > 1);
        
        gyro = new AHRS(NavXComType.kMXP_SPI);

        PoseEstimator = 
            new MecanumDrivePoseEstimator(
                Constants.kinematics,
                gyro.getRotation2d().unaryMinus(),
                getPosition(),
                Constants.InitialPose
            );

        mecanumDrive = new MecanumDrive(
            driveMotor[0].getMotor(),
            driveMotor[1].getMotor(),
            driveMotor[2].getMotor(),
            driveMotor[3].getMotor()
        );
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        mecanumDrive.driveCartesian(
            xLimiter.calculate(xSpeed),
            yLimiter.calculate(ySpeed),
            zLimiter.calculate(zRotation),
            gyro.getRotation2d().unaryMinus()
        );
    }

    public MecanumDriveWheelPositions getPosition() {
        return new MecanumDriveWheelPositions(
            driveMotor[0].getPosition(),
            driveMotor[1].getPosition(),
            driveMotor[2].getPosition(),
            driveMotor[3].getPosition()
        );
    }

    public static Drivetrain getInstance() {
        if(drivetrain == null) drivetrain = new Drivetrain();
        return drivetrain;
    }
}
