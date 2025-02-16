package frc.robot.subsystems.Candle2025;


// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;
import java.util.List;

import org.opencv.core.Mat.Tuple2;
import org.opencv.core.Mat.Tuple3;

import com.ctre.phoenix.led.*;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
// import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
// import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import frc.robot.Constants2025;

public class Candle2025 extends SubsystemBase {

    CANdle led = new CANdle(Constants2025.Candle.candleID, Constants2025.Candle.canBusName);

    private final int ledOffset = 8;
    private final int ledNum = 25;
    private final int slotNum = 2;
    private final Tuple2<Integer> leftPart = new Tuple2<Integer>(0, 11);
    private final Tuple2<Integer> rightPart = new Tuple2<Integer>(12, 24);
    enum COLOR {
        NONE,
        IDLE,
        CARRYING_CORAL,
        L1,
        L2,
        L3,
        L4,
        NUM,
    }

    private COLOR[][] slots = new COLOR[slotNum][ledNum];


    private final List<Tuple3<Integer>> palette = Arrays.asList(
        new Tuple3<>(0, 0, 0),
        new Tuple3<>(15, 79, 242),
        new Tuple3<>(15, 242, 238),
        new Tuple3<>(255, 255, 255),
        new Tuple3<>(235, 16, 16),
        new Tuple3<>(10, 247, 18),
        new Tuple3<>(168, 7, 222)
    );

    public Candle2025() {
        clear();
    }


    public void clear() {
        for (int i = 0; i < slotNum; ++i) {
            clearSlot(i);
        }

    }

    private void show() {
        // for (int i = 0; i < slotNum; ++i) {
        //     COLOR[] slot = slots[i];
        //     int start = 0;
        //     while (start < ledNum) {
        //         COLOR currentColor = slot[start];
        //         if (i > 0 && currentColor == COLOR.NONE) {
        //             start++;
        //             continue;
        //         }
        //         int end = start + 1;
        //         while (end < ledNum && slot[end] == currentColor) {
        //             end++;
        //         }
        //         Tuple3<Integer> color = palette.get(currentColor.ordinal());
        //         led.setLEDs(color.get_0().intValue(), color.get_1().intValue(), color.get_2().intValue(), 0,
        //                 start + ledOffset, end - start);
        //         start = end;
        //     }
        // }
        led.setLEDs(0, 0, 0);
    }

    public void showIdle() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.IDLE;
        }
    }

    public void showCarryingCoral() {
        int slotIdx = 0;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.CARRYING_CORAL;
        }
    }

    public void showL1() {
        clearLn();
        int slotIdx = 1;
        COLOR[] slot = slots[slotIdx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.L1;
        }
    }

    private void showLn(COLOR c, boolean isLeft) {
        clearLn();
        Tuple2<Integer> part = isLeft ? leftPart : rightPart;
        int slotIdx = 1;
        COLOR[] slot = slots[slotIdx];
        for (int i = part.get_0(); i <= part.get_1(); ++i) {
            slot[i] = COLOR.L1;
        }
    }

    public void showL2(boolean isLeft) {
        showLn(COLOR.L2, isLeft);
    }

    public void showL3(boolean isLeft) {
        showLn(COLOR.L3, isLeft);
    }

    public void showL4(boolean isLeft) {
        showLn(COLOR.L4, isLeft);
    }

    private void clearSlot(int idx) {
        if (idx < 0 || idx >= slotNum) {
            return;
        }
        COLOR[] slot = slots[idx];
        for (int i = 0; i < slot.length; ++i) {
            slot[i] = COLOR.NONE;
        }
    }


    public void clearLn() {
        clearSlot(1);
    }


    private void update() {
        show();
    }

    @Override
    public void periodic() {
        update();
    }

    public void init() {
        showIdle();
    }

    public void onDisable() {
        clear();
    }
}
