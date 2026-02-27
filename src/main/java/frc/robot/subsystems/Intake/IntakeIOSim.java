package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private final SingleJointedArmSim pivotSim;
    private final DCMotorSim intakeSim;
    private final DCMotor pivotMotor;
    private final DCMotor intakeMotor;
    private PIDController pivotController = new PIDController(0, 0, 0);
    private double pivotVoltage = 0;
    private double intakeVoltage = 0;
    private double desiredAngle;
    private boolean isPositionControl;
    private double kp, ki, kd, kv, ks;
    
    public IntakeIOSim(){
        pivotMotor = DCMotor.getKrakenX60(1);
        intakeMotor = DCMotor.getKrakenX44(1);

        pivotSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                pivotMotor, 
                SingleJointedArmSim.estimateMOI(
                    Inches.of(17).in(Meters), 
                    Pounds.of(7).in(Kilograms)
                ),
                27
            ), 
            pivotMotor, 
            27, 
            Inches.of(17).in(Meters), 
            Degrees.of(0).in(Radians), 
            Degrees.of(90).in(Radians),
            true, 
            Degrees.of(0).in(Radians)
        );

        intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                intakeMotor, 
                0.025, //TODO give this a real value, I made it up
                1
            ), 
            intakeMotor
        );
        
        desiredAngle = pivotSim.getAngleRads();
    }

    @Override
    public void updateIntakeInputs(PivotIOInputs pivotInputs, InfeedIOInputs infeedInputs) {
        Logger.recordOutput("Intake/Pivot Desired Angle", desiredAngle);

        pivotController.setPID(kp, ki, kd);

        if(isPositionControl){
            pivotVoltage = pivotController.calculate(pivotSim.getAngleRads(), desiredAngle);
        }
        
        pivotVoltage = MathUtil.clamp(pivotVoltage, -12, 12);
        intakeVoltage = MathUtil.clamp(intakeVoltage, -12, 12);

        pivotSim.setInputVoltage(pivotVoltage);
        intakeSim.setInputVoltage(intakeVoltage);

        pivotSim.update(0.02);
        intakeSim.update(0.02);

        pivotInputs.angle = pivotSim.getAngleRads();
        pivotInputs.velocity = pivotSim.getVelocityRadPerSec();
        pivotInputs.current = pivotSim.getCurrentDrawAmps();
        pivotInputs.voltage = pivotVoltage;

        infeedInputs.velocity = intakeSim.getAngularVelocityRPM();
        infeedInputs.current = intakeSim.getCurrentDrawAmps();
        infeedInputs.voltage = intakeVoltage;
    }

    @Override
    public void setPivotPosition(double angle) {
        isPositionControl = true;
        desiredAngle = Degrees.of(angle).in(Radians);
    }

    @Override
    public void setInfeedVelocity(double percent) {
        intakeVoltage = percent * 12;

    }

    @Override
    public void setPivotVoltage(double voltage) {
        isPositionControl = false;
        pivotVoltage = voltage;
    }

    @Override
    public void setInfeedVoltage(double voltage) {
        intakeVoltage = voltage;
    }
    
    @Override
    public void setPivotPID(double kp, double ki, double kd, double kv, double ks) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kv = kv;
        this.ks = ks;
    }

}
