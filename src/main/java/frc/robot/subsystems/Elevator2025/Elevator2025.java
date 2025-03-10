package frc.robot.subsystems.Elevator2025;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.BreakIterator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2025;
import frc.robot.subsystems.TurningArm2025.TurningArm2025.TA_STATE;
import frc.robot.utils.MiscUtils;

public class Elevator2025 extends SubsystemBase {
    public enum EV_STATE {
        NONE,
        ZERO,
        BASE,
        L1,
        L2,
        L3,
        L4,
        BALL1,
        BALL2,


        TUNING_UP,
        TUNING_DOWN,
    }
    public enum RUNNING_STATE {
        READY,
        RUNNING,
        DONE,
    }

    
    private String getFilePath() {
        if (RobotBase.isSimulation()) {
            return "d:/simulated_elevatorLastPosition.txt";
        } else {
            return "/home/lvuser/elevatorLastPosition.txt";
        }
    }

    private final double NONE_POS = -9999;
    private final double threshold = 0.05;

    private double elevatorOffset = 0;

    EV_STATE curState = EV_STATE.NONE;
    RUNNING_STATE curRunningState = RUNNING_STATE.READY;

    /** Creates a new ExampleSubsystem. */
    public Elevator2025() {
    }

    private final TalonFX m_primaryMotor = new TalonFX(Constants2025.Elevator.primaryMotorID, Constants2025.Elevator.canBusName);
    private final TalonFX m_followerMotor = new TalonFX(Constants2025.Elevator.followerMotorID, Constants2025.Elevator.canBusName);
    private final CANcoder m_canCoder = new CANcoder(Constants2025.Elevator.canCoderID, Constants2025.Elevator.canBusName);
    public MotionMagicVoltage motionMagicVoltage1 = new MotionMagicVoltage(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    // public MotionMagicVoltage motionMagicVoltage2 = new MotionMagicVoltage(1);
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    protected double lastMoveTargetPos = NONE_POS; 
    protected int curPidSlot = 0;
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private TalonFXConfiguration getMotorConfiguration(boolean isPrimary, boolean lockMotor) {
        TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();

        elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorConfiguration.MotorOutput.NeutralMode = lockMotor ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorConfiguration.Slot0.kP = Constants2025.Elevator.Up.KP;
        elevatorConfiguration.Slot0.kI = Constants2025.Elevator.Up.KI;
        elevatorConfiguration.Slot0.kD = Constants2025.Elevator.Up.KD;
        elevatorConfiguration.Slot0.kS = Constants2025.Elevator.KS;
        elevatorConfiguration.Slot0.kV = Constants2025.Elevator.KV;
        elevatorConfiguration.Slot0.kA = Constants2025.Elevator.KA;

        elevatorConfiguration.Slot1.kP = Constants2025.Elevator.Down.KP;
        elevatorConfiguration.Slot1.kI = Constants2025.Elevator.Down.KI;
        elevatorConfiguration.Slot1.kD = Constants2025.Elevator.Down.KD;

        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants2025.Elevator.Velocity;
        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = Constants2025.Elevator.Acceleration;
        elevatorConfiguration.MotionMagic.MotionMagicJerk = Constants2025.Elevator.Jerk;
        
        elevatorConfiguration.Feedback.FeedbackRemoteSensorID = Constants2025.Elevator.canCoderID;
        elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        elevatorConfiguration.Feedback.SensorToMechanismRatio = Constants2025.Elevator.SensorToMechanismRatio;
        elevatorConfiguration.Feedback.RotorToSensorRatio = Constants2025.Elevator.RotorToSensorRatio;
        return elevatorConfiguration;
    }

    // private CANcoderConfiguration getCCConfig() {
    //     CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    //     // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    //     cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //     cc_cfg.MagnetSensor.MagnetOffset = 0;

    //     return cc_cfg;
    // }

    boolean isFirstTimeInit = true;
    // int initCount = 0;
    public void init() {
        // SmartDashboard.putNumber("ccc init", m_canCoder.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("ccc init count", initCount);
        // Elevator.getConfigurator().apply(new TalonFXConfiguration());
        m_primaryMotor.getConfigurator().apply(getMotorConfiguration(true, true));
        m_followerMotor.getConfigurator().apply(getMotorConfiguration(false, true));

        m_followerMotor.setControl(new Follower(m_primaryMotor.getDeviceID(), false));
        // m_primaryMotor.setSafetyEnabled(true);      // 例子代码，不知道有什么用，不明白safetyEnabled是什么
        // m_canCode=gurator().apply(getCCConfig());

        if (isFirstTimeInit) {
            isFirstTimeInit = false;
            if (!loadLastPosition()) {
                m_canCoder.setPosition(0);
            }
        }

        motionMagicVoltage1 = new MotionMagicVoltage(0);

    }

    public void initInTestMode() {
        setState(EV_STATE.NONE);
        curRunningState = RUNNING_STATE.READY;
        m_primaryMotor.getConfigurator().apply(getMotorConfiguration(true, true));
        m_followerMotor.getConfigurator().apply(getMotorConfiguration(false, true));
        m_followerMotor.setControl(new Follower(m_primaryMotor.getDeviceID(), false));
        m_primaryMotor.stopMotor();
    }
    public void unlockMotor() {
        m_primaryMotor.getConfigurator().apply(getMotorConfiguration(true, false));
        m_followerMotor.getConfigurator().apply(getMotorConfiguration(false, false));
    }

    public void lockMotor() {
        m_primaryMotor.getConfigurator().apply(getMotorConfiguration(true, true));
        m_followerMotor.getConfigurator().apply(getMotorConfiguration(false, true));
        m_followerMotor.setControl(new Follower(m_primaryMotor.getDeviceID(), false));
    }

    public void setOffset(double v) {
        elevatorOffset = v;
    }

    public double getOffset() {
        return elevatorOffset;
    }

    public void clearOffset() {
        elevatorOffset = 0;
    }

    public void toggleTuningUp() {
        if (curState == EV_STATE.TUNING_UP) {
            setState(EV_STATE.NONE);
            // m_primaryMotor.stopMotor();
            driveDutyCycle.Output = 0;
            m_primaryMotor.setControl(driveDutyCycle);
        }
        else {
            setState(EV_STATE.TUNING_UP);
        }
    }

    public void toggleTuningDown() {
        if (curState == EV_STATE.TUNING_DOWN) {
            setState(EV_STATE.NONE);
            // m_primaryMotor.stopMotor();
            driveDutyCycle.Output = 0;
            m_primaryMotor.setControl(driveDutyCycle);
        }
        else {
            setState(EV_STATE.TUNING_DOWN);
        }
    }

    public void setState(EV_STATE stat) {
        if (curState == stat) {
            return;
        }
        curRunningState = RUNNING_STATE.RUNNING;
        curState = stat;
    }

    public EV_STATE getState() {
        return curState;
    }

    public RUNNING_STATE getCurRunningState() {
        return curRunningState;
    }

    public double getDifferFromTarget() {
        double pos = getStatePos(curState);
        return Math.abs(m_canCoder.getPosition().getValueAsDouble() - pos);
    }

    private boolean isDone(double targetPos) {
        if (Math.abs(m_canCoder.getPosition().getValueAsDouble() - targetPos) < threshold) {
            return true;
        }

        return false;
    }

    protected void updatePidSlot(double targetPos) {
        if (MiscUtils.compareDouble(lastMoveTargetPos, NONE_POS)) {
            lastMoveTargetPos = targetPos;
        }
        if (MiscUtils.compareDouble(lastMoveTargetPos, targetPos)) {
            return;
        }
        // System.out.println("-------------> " + targetPos + " ---> " + lastMoveTargetPos);
        if (isCurPosBelow(targetPos))
        // if (targetPos < lastMoveTargetPos) 
        {
            // up
            curPidSlot = 0;
        }
        else {
            // down
            curPidSlot = 1;
        }

        lastMoveTargetPos = targetPos;
    }

    protected void updateState() {
        SmartDashboard.putNumber("ELEVATOR ccc1", m_canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ELEVATOR ccc2", m_primaryMotor.getPosition().getValueAsDouble());
        
        // double pos = Constants2025.Elevator.basePos;
        // switch (curState) {
        //     case ZERO:
        //         pos = Constants2025.Elevator.zeroPos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         break;
        //     case BASE:
        //         pos = Constants2025.Elevator.basePos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         else {
        //             // System.out.println("---------EEEE: " + m_canCoder.getPosition().getValueAsDouble() + ", " + pos);
        //         }
        //         break;
        //     case L1:
        //         pos = Constants2025.Elevator.l1Pos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         break;
        //     case L2:
        //         pos = Constants2025.Elevator.l2Pos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         break;
        //     case L3:
        //         pos = Constants2025.Elevator.l3Pos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         break;
        //     case L4:
        //         pos = Constants2025.Elevator.l4Pos;
        //         if (isDone(pos)) {
        //             curRunningState = RUNNING_STATE.DONE;
        //         }
        //         break;
        //     case NONE:
        //         return;
        //     default:
        //         break;
        // }

        double output = 0.1;
        if (curState == EV_STATE.TUNING_UP) {
            // m_primaryMotor.setControl(driveVelocity.withVelocity(0.1));
            driveDutyCycle.Output = -output;
            m_primaryMotor.setControl(driveDutyCycle);
            SmartDashboard.putNumber("ELEVATOR ccc TUNING pos", m_canCoder.getPosition().getValueAsDouble());
            curRunningState = RUNNING_STATE.DONE;
            return;
        } else if (curState == EV_STATE.TUNING_DOWN) {
            // m_primaryMotor.setControl(driveVelocity.withVelocity(-0.1));
            driveDutyCycle.Output = output;
            m_primaryMotor.setControl(driveDutyCycle);
            SmartDashboard.putNumber("ELEVATOR ccc TUNING pos", m_canCoder.getPosition().getValueAsDouble());
            curRunningState = RUNNING_STATE.DONE;
            return;
        }
        double pos = getStatePos(curState);
        pos += elevatorOffset;
        SmartDashboard.putNumber("ELEVATOR ccc targetPos", pos);
        SmartDashboard.putString("ELEVATOR ccc curState", curState.name());
        SmartDashboard.putString("ELEVATOR ccc curRuningState", curRunningState.name());

        if (MiscUtils.compareDouble(pos, NONE_POS)) {
            return;
        }
        if (isDone(pos)) {
            curRunningState = RUNNING_STATE.DONE;
        }

        updatePidSlot(pos);
        SmartDashboard.putNumber("ELEVATOR ccc pidSlot", (int)curPidSlot);
        m_primaryMotor.setControl(motionMagicVoltage1.withPosition(pos).withSlot(curPidSlot));
    }
    

    // int upC = 0;
    // int dC = 0;

    // public void elevatorUp() {
    //     upC++;
    //     SmartDashboard.putNumber("EUp", upC);
    //     // motionMagicVoltage.Position = Constants2025.Elevator.Top;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants2025.TurningArm.Top));
    // }

    // public void elevatorDown() {
    //     dC++;
    //     SmartDashboard.putNumber("EDown", dC);
    //     // motionMagicVoltage.Position = Constants2025.Elevator.Bottom;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants2025.TurningArm.Bottom));
    // }

    // public static boolean elevatorTop() {
    //     if (m_arm.getPosition().getValueAsDouble() == Constants2025.TurningArm.Top)
    //         return true;
    //     else
    //         return false;
    // }

    private void saveLastPosition() {
        try {
            File file = new File(getFilePath());
            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter fileWriter = new FileWriter(file);
            fileWriter.write(String.valueOf(m_canCoder.getPosition().getValueAsDouble()));
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private boolean loadLastPosition() {
        try {
            File file = new File(getFilePath());
            if (!file.exists()) {
                return false;
            }
            FileReader fileReader = new FileReader(file);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            String line = bufferedReader.readLine();
            System.out.println("Elevator load canCoder position succssfully. line is : " + line);
            bufferedReader.close();
            fileReader.close();
            m_canCoder.setPosition(Double.parseDouble(line));
            return true;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return false;
    }

    public void onDisable() {
        saveLastPosition();
    }

    public void resetCancodePosition() {
        try {
            Files.delete(Path.of(getFilePath()));
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        m_canCoder.setPosition(0);
    }

    public double getStatePos(EV_STATE state) {
        switch (state) {
            case ZERO:
                return Constants2025.Elevator.zeroPos;
            case BASE:
                return Constants2025.Elevator.basePos;
            case L1:
                return Constants2025.Elevator.l1Pos;
            case L2:
                return Constants2025.Elevator.l2Pos;
            case L3:
                return Constants2025.Elevator.l3Pos;
            case L4:
                return Constants2025.Elevator.l4Pos;
            case BALL1:
                return Constants2025.Elevator.ball1Pos;
            case BALL2:
                return Constants2025.Elevator.ball2Pos;
            default:
                return NONE_POS;
        }
    }

    public double[] getDodgePosOrderFromUp2Down() {
        return new double[] {
            Constants2025.Elevator.upDodgePos,
            Constants2025.Elevator.downDodgePos,
        };
    }

    public double getCurPos() {
        return m_canCoder.getPosition().getValueAsDouble();
    }

    public boolean isCurPosBelow(double pos) {
        return isPosBelow(getCurPos(), pos);
    }

    public boolean isCurPosUpper(double pos) {
        return isPosUpper(getCurPos(), pos);
    }
    
    public boolean isPosBelow(double pos1, double pos2) {
        // smaller value means higher position
        return pos1 > pos2;
    }

    public boolean isPosUpper(double pos1, double pos2) {
        // smaller value means higher position
        return !isPosBelow(pos1, pos2);
    }
}
