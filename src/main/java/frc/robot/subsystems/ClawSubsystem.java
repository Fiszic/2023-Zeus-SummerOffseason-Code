package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase{
    private final Solenoid m_Solenoid;
    private CANSparkMax leftWheels;
    private CANSparkMax rightWheels;
    public ClawSubsystem(){
        m_Solenoid = new Solenoid(Constants.ID.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, Constants.ID.CLAW_PISTON);
        leftWheels = new CANSparkMax(Constants.ID.CLAW_LEFT, MotorType.kBrushless);
        rightWheels = new CANSparkMax(Constants.ID.CLAW_RIGHT, MotorType.kBrushless);
        leftWheels.restoreFactoryDefaults();
        rightWheels.restoreFactoryDefaults();

        leftWheels.setIdleMode(IdleMode.kBrake);
        rightWheels.setIdleMode(IdleMode.kBrake);

        rightWheels.setInverted(false);
        leftWheels.setInverted(true);

        leftWheels.setSmartCurrentLimit(22);
        rightWheels.setSmartCurrentLimit(22);

        // rightWheels.follow(leftWheels);

        leftWheels.clearFaults();
        rightWheels.clearFaults();


        setCoast();
        burnFlash();
        close();

    }
    public void close(){
        m_Solenoid.set(false);
    }
    public double getLCurrent(){
        return leftWheels.getOutputCurrent();
    }
    public double getRCurrent(){
        return rightWheels.getOutputCurrent();
    }
    public void open(){
        m_Solenoid.set(true);
    }
    public boolean getState(){
        return m_Solenoid.get();
    }

    public void setBrake(){
        leftWheels.setIdleMode(IdleMode.kBrake);
        rightWheels.setIdleMode(IdleMode.kBrake);
    }

    public void setCoast(){
        leftWheels.setIdleMode(IdleMode.kCoast);
        rightWheels.setIdleMode(IdleMode.kCoast);
    }
    
    public void burnFlash(){
        leftWheels.burnFlash();
        rightWheels.burnFlash();
    }

    public void setSpeed(double speed){
        // if(leftWheels.getOutputCurrent()>19||rightWheels.getOutputCurrent()>19){
        //     leftWheels.set(0);
        //     rightWheels.set(0);
        // }
        leftWheels.set(speed);
        rightWheels.set(speed);
    }

    public void setVoltage(double volt){
        leftWheels.setVoltage(volt);
        rightWheels.setVoltage(volt);
    }

    public void stop(){
        leftWheels.stopMotor();
        rightWheels.stopMotor();
        close();
    }
}
