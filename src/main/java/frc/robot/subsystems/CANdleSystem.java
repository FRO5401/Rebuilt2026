/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSystem extends SubsystemBase {
    private final int LEDS_PER_ANIMATION = 100;
    private final CANdle m_candle = new CANdle(19);
    // private XboxController joystick;
    private int m_candleChannel = 8;
    private boolean m_animDirection = false;

    public enum AnimationTypes {
        ColorFlow,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        AboutToShift,
        Twinkle,
        TwinkleOff,
        SetAll,
        Looking,
        Align,
        Climb
    }


    public CANdleSystem() {
        // this.joystick = joy;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.withLED(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(.8));
        configAll.withCANdleFeatures(
                new CANdleFeaturesConfigs().withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
                        .withEnable5VRail(Enable5VRailValue.Enabled).withVBatOutputMode(VBatOutputModeValue.Modulated));

        m_candle.getConfigurator().apply(configAll);
        changeAnimation(AnimationTypes.Larson);
    }

    // public void toggle5VOverride() {
    // System.out.println("State is: " + m_last5V);
    // m_candle.configV5Enabled(m_last5V);
    // m_last5V = !m_last5V;
    // }

    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }

    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount().getValue();
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getVBatModulation().getValue();
    }

    public double get5V() {
        return m_candle.getSupplyVoltage().getValueAsDouble();
    }

    public double getCurrent() {
        return m_candle.getOutputCurrent().getValueAsDouble();
    }

    public double getTemperature() {
        return m_candle.getDeviceTemp().getValueAsDouble();
    }

    public void changeAnimation(AnimationTypes toChange) {

        switch (toChange) {
            default:
            case ColorFlow:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION));
                break;

            case Larson:
                m_candle.setControl(new LarsonAnimation(m_candleChannel, LEDS_PER_ANIMATION)
                        .withColor(RGBWColor.fromHex("#042698").get()));
                break;

            case Rainbow:
                m_candle.setControl(new RainbowAnimation(m_candleChannel, LEDS_PER_ANIMATION));
                break;

            case RgbFade:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case SingleFade:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case Strobe:
                m_candle.setControl(new StrobeAnimation(m_candleChannel, LEDS_PER_ANIMATION)
                        .withColor(RGBWColor.fromHex("#9000ffff").get()));
                break;

            case Twinkle:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case TwinkleOff:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case Climb:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case Looking:
                m_candle.setControl(new SingleFadeAnimation(m_candleChannel, LEDS_PER_ANIMATION)
                        .withColor(RGBWColor.fromHex("#000000").get()));
                break;

            case Align:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

            case SetAll:
                m_candle.setControl(new ColorFlowAnimation(m_candleChannel, LEDS_PER_ANIMATION).withColor(null));
                break;

        }
    }

    public void clearAllAnims() {

    }

    @Override
    public void periodic() {

    }

    public Command setLights(AnimationTypes toChange) {
        return runOnce(() -> {
            clearAllAnims();
            changeAnimation(toChange);
        });
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}