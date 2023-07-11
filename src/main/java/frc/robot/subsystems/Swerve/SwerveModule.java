package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    public final WPI_TalonFX driver;
    private final WPI_TalonFX rotator;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    private final double driverEncoderRatio = (1/2048.0)*(1/6.55)*(0.1016*Math.PI); //4 inch to 0.1016 meters
    private final double rotatorEncoderRatio = (1/2048.0)*(1/10.29)*(2*Math.PI);

    private final double maxDriverSpeed = Constants.RobotInfo.MAX_ROBOT_SPEED; //change
    private final StatorCurrentLimitConfiguration StatorCurrentLimitD;
    // private final SupplyCurrentLimitConfiguration SupplyCurrentLimitD;
    // private final StatorCurrentLimitConfiguration StatorCurrentLimitR;
    // private final SupplyCurrentLimitConfiguration SupplyCurrentLimitR;

    private final PIDController rotatorPID;
    public SwerveModule(int driverID, int rotatorID, boolean driveMotorReversed, boolean rotatorMotorReversed,
    int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        driver = new WPI_TalonFX(driverID, "DriveTrain");
        
        rotator = new WPI_TalonFX(rotatorID, "DriveTrain");
        driver.configFactoryDefault();
        rotator.configFactoryDefault();
        driver.setInverted(driveMotorReversed);
        rotator.setInverted(rotatorMotorReversed);
        setCoastMode();
        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        StatorCurrentLimitD = new StatorCurrentLimitConfiguration(true, 33, 32, 0.01);
        // SupplyCurrentLimitD = new SupplyCurrentLimitConfiguration(true, 32, 31, 0.01);
        // StatorCurrentLimitR = new StatorCurrentLimitConfiguration(true, 32, 31, 0.01);
        // SupplyCurrentLimitR = new SupplyCurrentLimitConfiguration(true, 32, 31, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimitD);
        // driver.configSupplyCurrentLimit(SupplyCurrentLimitD);
        // rotator.configStatorCurrentLimit(StatorCurrentLimitR);
        // rotator.configSupplyCurrentLimit(SupplyCurrentLimitR);

       

        absoluteEncoder = new CANCoder(absoluteEncoderID, "DriveTrain");
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        rotatorPID = new PIDController(0.425, 0.25, 0);//change 0.425 0.28
        rotatorPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public void setAutonCurrent(){
         StatorCurrentLimitConfiguration StatorCurrentLimitD = new StatorCurrentLimitConfiguration(false, 33, 32, 0.01);
         driver.configStatorCurrentLimit(StatorCurrentLimitD);
        }
    public void setTeleopCurrent(){
        StatorCurrentLimitConfiguration StatorCurrentLimitD = new StatorCurrentLimitConfiguration(true, 33, 32, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimitD);
    }
    public double getDriverEncoderDistance(){
        return driver.getSelectedSensorPosition()*driverEncoderRatio;
    }
    public double getDriverEncoderVelocity(){
        return driver.getSelectedSensorVelocity()*driverEncoderRatio;
    }
    public double getRotatorEncoderRotation(){
        return rotator.getSelectedSensorPosition()*rotatorEncoderRatio;
    }
    public double getRotatorEncoderVelocity(){
        return rotator.getSelectedSensorVelocity()*rotatorEncoderRatio;
    }
    public double getAbsoluteEncoderRotation(){
        double pos =absoluteEncoder.getAbsolutePosition()-absoluteEncoderOffset;
        pos = Math.toRadians(pos);
        return pos*(absoluteEncoderReversed?-1:1);

    }
    public double getAbsoluteEncoderRotationDegrees(){
        double pos =absoluteEncoder.getAbsolutePosition()-absoluteEncoderOffset;
        // pos = Math.toRadians(pos);
        return pos*(absoluteEncoderReversed?-1:1);

    }
    public double getRawAbsoluteEncoderPosition(){
        return absoluteEncoder.getAbsolutePosition();
    }
    public void resetEncoders(){
        resetDriveEncoder(0);
        resetRotatorEncoder(getAbsoluteEncoderRotation());
    }
    public void resetDriveEncoder(double pos){
        driver.setSelectedSensorPosition(pos);
    }
    public void resetRotatorEncoder(double pos){
        rotator.setSelectedSensorPosition(pos/rotatorEncoderRatio);
    }

    public void setBrakeMode(){
        driver.setNeutralMode(NeutralMode.Brake);
        rotator.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode(){
        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriverEncoderVelocity(), new Rotation2d(getRotatorEncoderRotation()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriverEncoderDistance(), new Rotation2d(getRotatorEncoderRotation()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        SmartDashboard.putNumber("ActualAngle ", state.angle.getDegrees());
        SmartDashboard.putNumber("DesiredAngle", getState().angle.getDegrees());
        driver.set(state.speedMetersPerSecond/maxDriverSpeed);
        rotator.set(rotatorPID.calculate(getRotatorEncoderRotation(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+driver.getDeviceID()+"] state",state.toString());

    }
    public void stop(){
        driver.set(0);
        rotator.set(0);
    }
}
