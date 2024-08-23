package frc.robot.subsystems.Candle;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.Constants;

public class Candle extends SubsystemBase {

    CANdle led = new CANdle(Constants.Candle.candleID);

    boolean isOn = false;
    int r = 0;
    int g = 0;
    int b = 0;

    enum Mode {
        Hold,
        BlinkThenHold,
    }

    Mode mode = Mode.Hold;
    double blinkStartTime = 0;
    double BLINK_ON_DURATION = 0.1;
    double BLINK_OFF_DURATION = 0.05;
    double BLINE_ROUND_TIME = BLINK_ON_DURATION + BLINK_OFF_DURATION;
    double BLINK_TIMES = 10;

    public Candle() {

    }

    public void white() {
        mode = Mode.Hold;
        isOn = true;
        r = 255;
        g = 255;
        b = 255;
    }

    public void yellow() {
        mode = Mode.Hold;
        isOn = true;
        r = 255;
        g = 255;
        b = 0;
    }

    public void red() {
        mode = Mode.Hold;
        isOn = true;
        r = 255;
        g = 0;
        b = 0;
    }

    public void blue() {
        mode = Mode.Hold;
        isOn = true;
        r = 0;
        g = 0;
        b = 255;
    }

    public void blinkGreenThenHold() {
        if (mode == Mode.BlinkThenHold) {
            return;
        }
        mode = Mode.BlinkThenHold;
        isOn = true;
        r = 0;
        g = 255;
        b = 0;
        blinkStartTime = Timer.getFPGATimestamp();
    }

    public void off() {
        isOn = false;
    }

    public void update() {
        if (isOn) {
            switch (mode) {
                case Hold: {
                    led.setLEDs(r, g, b);
                    break;
                }
                case BlinkThenHold: {
                    double now = Timer.getFPGATimestamp();
                    double dt = now - blinkStartTime;
                    long round = Math.round(dt / BLINE_ROUND_TIME);
                    if (round >= BLINK_TIMES) {
                        mode = Mode.Hold;
                    } else {
                        double dur = dt;
                        while (dur >= BLINE_ROUND_TIME) {
                            dur -= BLINE_ROUND_TIME;
                        }
                        if (dur <= BLINK_ON_DURATION) {
                            led.setLEDs(r, g, b);
                        } else {
                            led.setLEDs(0, 0, 0);
                        }
                    }
                    break;
                }

                default:
                    break;
            }
        } else {
            led.setLEDs(0, 0, 0);
        }
    }

    @Override
    public void periodic() {
        update();
        // SmartDashboard.putBoolean("Led is On", isOn);
        // SmartDashboard.putString("Led Mode", mode.toString());
    }
}
