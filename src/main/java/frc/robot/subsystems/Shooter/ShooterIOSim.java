package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {

    private final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
    private final PIDController controller = new PIDController(ShooterConstants.KP_SIM, ShooterConstants.KI_SIM, ShooterConstants.KD_SIM);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motorModel, 0.025, ShooterConstants.GEAR_RATIO), motorModel);


    public ShooterIOSim(){
        
    }

    public void updateInputs(ShooterIOInputs inputs){
        inputs.velocity = driveSim.getAngularVelocityRPM();
        inputs.voltage = driveSim.getInputVoltage();
        inputs.current = driveSim.getCurrentDrawAmps();
        driveSim.update(.02);
    }

    public void applyVoltage(double voltage){
        double appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(appliedVoltage);
    }

    public void setVelocity(double velocity){
        driveSim.setAngularVelocity(velocity);
    }

    public void stop(){
        driveSim.setInputVoltage(0);
    }

}
