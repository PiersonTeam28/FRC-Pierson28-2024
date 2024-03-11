package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class PizzaBox extends SubsystemBase {
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;
    private TalonFX holdingMotor;

    public PizzaBox() {
        leftShooterMotor = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
        rightShooterMotor.follow(leftShooterMotor, true);
        holdingMotor = new TalonFX(19);

    }

    public void startShootMotors(){
        leftShooterMotor.setVoltage(12);
    }

    public void startIntakeMotors(){
        System.out.println("intaking");
        leftShooterMotor.setVoltage(-5);
        holdingMotor.setVoltage(-5);
    }

    public void startDispenseMotors(){
        leftShooterMotor.setVoltage(3);
        holdingMotor.setVoltage(3);
    }

    public void releaseHoldingMotor(){
        holdingMotor.setVoltage(3);
    }

    public void stopAllMotors(){
        System.out.println("stop");
        leftShooterMotor.stopMotor();
        holdingMotor.stopMotor();
    }

    public ShootCommand shoot(){
        return new ShootCommand(this);
    }

    public InstantCommand intake(){
        return new InstantCommand(() -> this.startIntakeMotors(), this);
    }

    public InstantCommand stop(){
        return new InstantCommand(() -> this.stopAllMotors(), this);
    }

    public InstantCommand dispense(){
        return new InstantCommand(() -> this.startDispenseMotors());
    }
    
    @Override
    public void periodic(){
        
    }
}