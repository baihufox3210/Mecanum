package frc.robot.Drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MecanumModule {
    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    public MecanumModule(int MotorID, boolean driveInverted) {
        driveMotor = new SparkMax(MotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(Constants.MotorMode)
            .inverted(driveInverted)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        driveConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        driveMotor.configure(
            driveConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public double getPosition() {
        return driveEncoder.getPosition();
    }

    public SparkMax getMotor() {
        return driveMotor;
    }
}
