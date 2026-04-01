package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TurretConstants;

public class TurretIOSim implements TurretIO {
    private double desiredPos = 0;
    private PIDController controller = RobotBase.isReal()
            ? new PIDController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD)
            : new PIDController(TurretConstants.KP_SIM, TurretConstants.KI_SIM, TurretConstants.KD_SIM);

    private final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.025, TurretConstants.GEAR_RATIO),
            motorModel);

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        applyVoltage(controller.calculate(inputs.position, desiredPos));

        inputs.position = driveSim.getAngularPositionRotations();
        inputs.velocity = driveSim.getAngularVelocityRPM();
        inputs.temperature = 0;
        inputs.voltage = driveSim.getInputVoltage();
        inputs.current = driveSim.getCurrentDrawAmps();
        inputs.applied = 0;
        driveSim.update(.02);
    }

    @Override
    public void applyVoltage(double voltage) {
        double appliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void applyDutyCycle(double percent) {
        double appliedVoltage = MathUtil.clamp(percent, -12.0, 12.0);
        driveSim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stop() {
        driveSim.setInputVoltage(0);
    }

    @Override
    public void setPosition(double position) {
        desiredPos = position;
    }

}
