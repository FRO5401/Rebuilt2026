package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import frc.robot.Constants.HoodConstants;

public class HoodIOTalonFX implements HoodIO {

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID);
  private final CANcoder hoodCANcoder = new CANcoder(HoodConstants.HOOD_ENCODER_ID);
  private final FeedbackConfigs feedbackConfig = HoodConstants.FEEDBACK_CONFIGS;

  public HoodIOTalonFX() {
    hoodCANcoder.setPosition(0);
    hoodCANcoder.getConfigurator().apply(HoodConstants.ENCODER_CONFIG);

    feedbackConfig
        .withRemoteCANcoder(hoodCANcoder)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

    HoodConstants.CONFIG.withFeedback(feedbackConfig);

    hoodMotor.getConfigurator().apply(HoodConstants.CONFIG);
    hoodMotor.setPosition(0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.motorPosition = hoodMotor.getPosition().getValueAsDouble();
    inputs.temperature = hoodMotor.getDeviceTemp().getValueAsDouble();
    inputs.motorVelocity = hoodMotor.getVelocity().getValueAsDouble();

    inputs.wantedPosition = hoodMotor.getClosedLoopReference().getValueAsDouble();
    inputs.encoderPosition = hoodCANcoder.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    hoodMotor.setControl(positionRequest.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void stop() {
    hoodMotor.stopMotor();
  }

  @Override
  public void setPID(double p, double i, double d, double ks) {
    HoodConstants.CLOSED_LOOP.kP = p;
    HoodConstants.CLOSED_LOOP.kI = i;
    HoodConstants.CLOSED_LOOP.kD = d;
    HoodConstants.CLOSED_LOOP.kS = ks;
    hoodMotor.getConfigurator().apply(HoodConstants.CLOSED_LOOP);
  }
}
