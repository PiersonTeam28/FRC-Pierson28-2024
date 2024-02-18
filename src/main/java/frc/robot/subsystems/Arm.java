package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX motor;
    

    public Arm() {
        // TO DO: Perfect slot configs for motor control
        motor = new TalonFX(-4);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 24;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.1;
        motor.getConfigurator().apply(slot0Configs);
        motor.setPosition(0);
    }

    public void moveToPose(double pose){
        motor.setControl(new PositionVoltage(pose, .1, false, 0, 0, false, false, false));
    }

    public void down(){
        motor.setVoltage(2);
    }

    public void up(){
        motor.setVoltage(-2);
    }
    
    @Override
    public void periodic(){
        
    }
}