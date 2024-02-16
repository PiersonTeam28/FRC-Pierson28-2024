package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class PizzaBox extends SubsystemBase {
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final TalonFX holdingMotor;

    public PizzaBox() {
        leftShooterMotor = new CANSparkMax(-1, CANSparkLowLevel.MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(-2, CANSparkLowLevel.MotorType.kBrushless);
        rightShooterMotor.follow(leftShooterMotor);
        rightShooterMotor.setInverted(true);
        holdingMotor = new TalonFX(-3);

    }

    public void intake(){

    }

    public void shoot(){

    }

    public void dispense(){

    }
    
    
    @Override
    public void periodic(){
        
    }
}