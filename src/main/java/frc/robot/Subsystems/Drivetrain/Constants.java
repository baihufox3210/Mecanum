package frc.robot.Subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class Constants {
    public static final int MotorID[] = {11, 12, 13, 14};

    // 待修正
    public static final double GearRatio = 10.71;
    public static final double WheelCirc = Inches.of(6).times(Math.PI).in(Meters);

    public static final double PositionConversionFactor = 1 / GearRatio * WheelCirc;
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;

    // 待修正
    public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(-0.285, 0.26),
        new Translation2d(-0.285, -0.26),
        new Translation2d(0.285, 0.26),
        new Translation2d(0.285, -0.26)
    );

    public static final double SpeedLimiter = 1.5;

    public static final Pose2d InitialPose = new Pose2d(0, 0, Rotation2d.kZero);

    public static final IdleMode MotorMode = IdleMode.kBrake;
    public static final int SlipCurrent = 44;

    public static final PIDController xPID = new PIDController(1, 0.0, 0.0);
    public static final PIDController yPID = new PIDController(1, 0.0, 0.0);
    public static final PIDController zPID = new PIDController(1, 0.0, 0.0);

    public static final PIDController turnPID = new PIDController(0.01, 0.0, 0.0);
}
