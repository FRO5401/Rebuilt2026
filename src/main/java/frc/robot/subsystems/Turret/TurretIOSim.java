package frc.robot.subsystems.Turret;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {

    private final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);


      private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(motorModel, 0.025, TurretConstants.GEAR_RATIO),
          motorModel);

    public TurretIOSim(){
        
    }

    public void updateInputs(TurretIOInputs inputs){
        inputs.position = driveSim.getAngularPositionRotations();
        inputs.velocity = driveSim.getAngularVelocityRPM();
        inputs.temperature = 0;
        inputs.voltage = driveSim.getInputVoltage();
        inputs.current = driveSim.getCurrentDrawAmps();
        driveSim.update(.02);
    }

    public void applyVoltage(double voltage){
        double appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(appliedVoltage);
    }

    public void stop(){
        driveSim.setInputVoltage(0);
    }

}
