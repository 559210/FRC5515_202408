package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControlPadHelper.TopicWrap.TopicType;
import frc.robot.utils.MiscUtils;

public class ControlPadHelper {
    public static class DebugCtrl {
        // the code below is from ControlPad Unity project.
        private enum DebugPanelInfo
        {
            NONE,
            ZERO,
            READY_FOR_LOAD_CORAL,
            L1,
            L2,
            L3,
            L4,
            BALL1,
            BALL2,
            UP_ARROW,
            DOWN_ARROW,
            LEFT_ARROW,
            RIGHT_ARROW,
            SHOOT,
        }
        static public long val = -1;
        static public boolean isLoadCoral = false;
        static public boolean isLoadBall = false;
        // static public Trigger onZero = new Trigger(eventLoop, () -> val == DebugPanelInfo.ZERO.ordinal());
        static public Trigger onReadyForloadCoral = new Trigger(eventLoop, () -> val == DebugPanelInfo.READY_FOR_LOAD_CORAL.ordinal());
        static public Trigger onL1 = new Trigger(eventLoop, () -> val == DebugPanelInfo.L1.ordinal());
        static public Trigger onL2 = new Trigger(eventLoop, () -> val == DebugPanelInfo.L2.ordinal());
        static public Trigger onL3 = new Trigger(eventLoop, () -> val == DebugPanelInfo.L3.ordinal());
        static public Trigger onL4 = new Trigger(eventLoop, () -> val == DebugPanelInfo.L4.ordinal());
        static public Trigger onBall1 = new Trigger(eventLoop, () -> val == DebugPanelInfo.BALL1.ordinal());
        static public Trigger onBall2 = new Trigger(eventLoop, () -> val == DebugPanelInfo.BALL2.ordinal());
        // static public Trigger onLoadCoral = new Trigger(eventLoop, () -> isLoadCoral);
        // static public Trigger onLoadBall = new Trigger(eventLoop, () -> isLoadBall);

        static public Trigger up = new Trigger(eventLoop, () -> val == DebugPanelInfo.UP_ARROW.ordinal());
        static public Trigger down = new Trigger(eventLoop, () -> val == DebugPanelInfo.DOWN_ARROW.ordinal());
        static public Trigger left = new Trigger(eventLoop, () -> val == DebugPanelInfo.LEFT_ARROW.ordinal());
        static public Trigger right = new Trigger(eventLoop, () -> val == DebugPanelInfo.RIGHT_ARROW.ordinal());
        static public Trigger shoot = new Trigger(eventLoop, () -> val == DebugPanelInfo.SHOOT.ordinal());
    }   
    public static class ControlPadInfo {
        public static class ControlPadInfoData {
            public long aprilTagId = -1;
            public long level = 0;
            public long branch = -1;
        }
        public ControlPadInfoData data = new ControlPadInfoData();
        ControlPadInfoData backupData = new ControlPadInfoData();
        ControlPadInfoData autoData = new ControlPadInfoData();

        public boolean isChanged = false;

        public void compareOld() {
            isChanged = false;
            if (data.aprilTagId != backupData.aprilTagId || data.level != backupData.level || data.branch != backupData.branch) {
                isChanged = true;
            }
        }

        public void backup() {
            backupData.aprilTagId = data.aprilTagId;
            backupData.level = data.level;
            backupData.branch = data.branch;
        }
    }
    public static class VirtualControl {
        public double x = 0.0;
        public double y = 0.0;
        public boolean isTap = false;
    }

    public static class SetHeadData {
        public boolean isSetHead = false;
        public long frameCount = 0; // no use, just a timestamp like frame count
    }

    protected static class TopicWrap {
        public enum ActionType {
            SUB,    // Subscribe
            PUB,    // Publish
        }
        public enum TopicType {
            INT_ARRAY,
            DOUBLE_ARRAY,
            DOUBLE,
        }

        private static final long[] nullLongArray = new long[0];
        private static final double[] nullDoubleArray = new double[0];
        
        private IntegerArrayTopic intArrayTopic = null;
        private IntegerArrayEntry intArrayEntry = null;
        private IntegerArrayPublisher intArrayPublisher = null;
        private DoubleArrayTopic doubleArrayTopic = null;
        private DoubleArrayEntry doubleArrayEntry = null;
        private DoubleArrayPublisher doubleArrayPublisher = null;
        private DoubleTopic doubleTopic = null;
        private DoubleEntry doubleEntry = null;
        private DoublePublisher doublePublisher = null;
        private TopicType topicType;
        private ActionType actionType;
        private String entryName;


        int topicListenerHandle = -1;


        public TopicWrap(TopicType type, String entryName, ActionType actionType) {
            topicType = type;
            this.actionType = actionType;
            this.entryName = entryName;

            switch (topicType) {
                case INT_ARRAY:
                    {
                        intArrayTopic = getNTTable().getIntegerArrayTopic(this.entryName);
                        if (actionType == ActionType.SUB) {
                            intArrayEntry = intArrayTopic.getEntry(nullLongArray);
                        }
                        else if (actionType == ActionType.PUB) {
                            intArrayPublisher = intArrayTopic.publish(PubSubOption.keepDuplicates(true));
                        }

                        // topicListenerHandle = getNTInst().addListener(
                        //     intArrayEntry,
                        //     EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        //     event -> {
                        //         isUpdated = true;
                        //     });
                    }
                    break;
                case DOUBLE_ARRAY:
                    {
                        doubleArrayTopic = getNTTable().getDoubleArrayTopic(this.entryName);
                        if (actionType == ActionType.SUB) {
                            doubleArrayEntry = doubleArrayTopic.getEntry(nullDoubleArray);
                        }
                        else if (actionType == ActionType.PUB) {
                            doubleArrayPublisher = doubleArrayTopic.publish(PubSubOption.keepDuplicates(true));
                        }
                    }
                    break;
                case DOUBLE:
                    {
                        doubleTopic = getNTTable().getDoubleTopic(this.entryName);
                        if (actionType == ActionType.SUB) {
                            doubleEntry = doubleTopic.getEntry(0.0);
                        }
                        else if (actionType == ActionType.PUB) {
                            doublePublisher = doubleTopic.publish(PubSubOption.keepDuplicates(true));
                        }
                    }
                default:
                    break;
            }
        }


        public double[] getDoubleArrayValue() {
            if (doubleArrayEntry != null) {
                // doubleArrayEntry.
                return doubleArrayEntry.get();
            }
            return null;
        }

        public void publishDoubleArray(double[] datas) {
            if (doubleArrayPublisher != null) {
                doubleArrayPublisher.set(datas);
                Flush();
            }
        }

        public long[] getIntArrayValue() {
            if (intArrayEntry != null) {
                return intArrayEntry.get();
            }
            return null;
        }

        public void publishIntArray(long[] datas) {
            if (intArrayPublisher != null) {
                intArrayPublisher.set(datas);
                Flush();
            }
        }

        public double getDoubleValue() {
            if (doubleEntry != null) {
                return doubleEntry.get();
            }
            // throw new RuntimeException("DoubleEntry is null");
            return 0.0;
        }

        public void publishDouble(double data) {
            if (doublePublisher != null) {
                doublePublisher.set(data);
                Flush();
            }
        }
    }
    private static final String tableName = "5515ControlPad";

    private static ControlPadInfo controlPadInfo = new ControlPadInfo();
    private static VirtualControl virtualControl = new VirtualControl();
    private static SetHeadData setHeadData = new SetHeadData();

    private static EventLoop eventLoop = new EventLoop();
    public static Trigger tapTrigger = new Trigger(eventLoop, () -> tapTriggerSupplier());
    public static Trigger setHeadTrigger = new Trigger(eventLoop, () -> setHeadSupplier());

    private static NetworkTableInstance ntInst = null;
    private static NetworkTable ntTable = null;
    private static TopicWrap controlPadInfoTopic = new TopicWrap(TopicType.INT_ARRAY, "ControlPadInfo", TopicWrap.ActionType.SUB);
    private static TopicWrap virtualControlTopic = new TopicWrap(TopicType.DOUBLE_ARRAY, "VirtualControl", TopicWrap.ActionType.SUB);
    private static TopicWrap setHeadTopic = new TopicWrap(TopicType.INT_ARRAY, "SetHead", TopicWrap.ActionType.SUB);
    private static TopicWrap robotPosTopic = new TopicWrap(TopicType.DOUBLE_ARRAY, "RobotPos", TopicWrap.ActionType.PUB);
    private static TopicWrap controlPadInfoRecallTopic = new TopicWrap(TopicType.INT_ARRAY, "ControlPadInfoRecall", TopicWrap.ActionType.PUB);

    private static TopicWrap debugCtrlTopic = new TopicWrap(TopicType.INT_ARRAY, "Debug", TopicWrap.ActionType.SUB);

    private static NetworkTableInstance getNTInst() {
        if (ntInst != null) {
            return ntInst;
        }

        ntInst = NetworkTableInstance.getDefault();
        return ntInst;
    }
    private static NetworkTable getNTTable() {
        if (ntTable != null) {
            return ntTable;
        }
        ntTable = getNTInst().getTable(tableName);
        return ntTable;
    }

    private static void Flush() {
        getNTInst().flush();
    }

    public static void publishRobotPos(Pose2d pos) {
        robotPosTopic.publishDoubleArray(new double[]{pos.getTranslation().getX(), pos.getTranslation().getY(), pos.getRotation().getDegrees()});
    }

    private static void publishControlPadInfoRecall() {
        if (controlPadInfo.data.aprilTagId == -1) {
            long[] datas = new long[]{-1, -1, -1};
            controlPadInfoRecallTopic.publishIntArray(datas);
            return;
        }
        long[] datas = new long[]{controlPadInfo.data.aprilTagId, controlPadInfo.data.level, controlPadInfo.data.branch};
        controlPadInfoRecallTopic.publishIntArray(datas);
    }

    private static void refreshControlPad() {
        long[] datas = controlPadInfoTopic.getIntArrayValue();
        // datas[0] is apriltag id
        // datas[2] is level of branch. 0 is bottom, 1 is 1st level, 2 is 2nd level, 3 is 3rd level
        // datas[1] is the left or right branch. -1 is left, 1 is right, 0 means level is bottom
        if (datas.length == 0) {
            return;
        }
        controlPadInfo.backup();
        controlPadInfo.data.aprilTagId = datas[0];
        controlPadInfo.data.level = datas[1];
        controlPadInfo.data.branch = datas[2];
        controlPadInfo.compareOld();

        SmartDashboard.putNumber("ControlPad aprilTagId", controlPadInfo.data.aprilTagId);
        SmartDashboard.putNumber("ControlPad level", controlPadInfo.data.level);
        SmartDashboard.putNumber("ControlPad branch", controlPadInfo.data.branch);
    }

    private static void refreshVirtualControl() {
        double[] datas = virtualControlTopic.getDoubleArrayValue();
        if (datas.length == 0) {
            return;
        }
        virtualControl.isTap = MiscUtils.compareDouble(datas[0], 1.0);
        virtualControl.x = datas[1];
        virtualControl.y = datas[2];
    }

    private static void refreshSetHead() {
        long[] datas = setHeadTopic.getIntArrayValue();
        if (datas.length == 0) {
            return;
        }
        setHeadData.isSetHead = datas[0] == 1;
        setHeadData.frameCount = datas[1];
    }

    public static ControlPadInfo.ControlPadInfoData getControlPadInfo() {
        if (Robot.inst.isAutonomous()) {
            return controlPadInfo.autoData;
        }
        
        if (controlPadInfo.data.aprilTagId == -1) {
            return null;
        }
        return controlPadInfo.data;
    }

    public static ControlPadInfo.ControlPadInfoData getControlPadInfoInAuto() {
        return controlPadInfo.autoData;
    }

    public static void setControlPadInfoDataInAuto(long aprilTagId, boolean isLeft, long level) {
        controlPadInfo.autoData.aprilTagId = aprilTagId;
        controlPadInfo.autoData.branch = isLeft ? -1 : 1;
        controlPadInfo.autoData.level = level;
    }

    public static VirtualControl getVirtualControl() {
        return virtualControl;
    }

    private static boolean tapTriggerSupplier() {
        return virtualControl.isTap;
    }

    private static boolean setHeadSupplier() {
        return setHeadData.isSetHead;
    }

    private static void refreshDebugCtrl() {
        long[] datas = debugCtrlTopic.getIntArrayValue();
        if (datas.length == 0) {
            return;
        }

        if (DebugCtrl.val != datas[0]) {
            System.out.println("========================> DebugCtrl: " + datas[0]);
        }
        DebugCtrl.val = datas[0];
        DebugCtrl.isLoadCoral = datas[1] == 1;
        DebugCtrl.isLoadBall = datas[2] == 1;
    }

    public static void init() {
        // getNTTable().addListener(EnumSet.of(NetworkTableEvent.Kind.kTopic), (nt, s, event) -> {
        //     System.out.println("------------------------> EVENT");
        // });

    }

    public static void update() {
        refreshControlPad();
        refreshVirtualControl();
        publishControlPadInfoRecall();
        refreshSetHead();
        refreshDebugCtrl();
        eventLoop.poll();
    }
}
