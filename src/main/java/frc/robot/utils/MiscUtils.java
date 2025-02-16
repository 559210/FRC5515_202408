package frc.robot.utils;

public final class MiscUtils {
    public static boolean isAllZero(double[] arr) {
        for (int i = 0; i < arr.length; ++i) {
            double n = arr[i];
            if (Math.abs(n - 0.0) > 1e-6) {
                return false;
            }
        }

        return true;
    }

    public static double EPSILON = 1e-6;
    public static float EPSILONF = 1e-6f;

    public static boolean compareDouble(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static boolean compareFloat(float a, float b) {
        return Math.abs(a - b) < EPSILONF;
    }

    public static double calcAngle2Point(double x1, double y1, double x2, double y2) {
        // 假设a和b的坐标
        // double x1 = ...; // 填入a点的x坐标
        // double y1 = ...; // 填入a点的y坐标
        // double x2 = ...; // 填入b点的x坐标
        // double y2 = ...; // 填入b点的y坐标

        // 计算连线ab与x轴正方向的夹角（弧度）
        double angleRad = Math.atan2(y2 - y1, x2 - x1);

        // 将弧度转换为度数
        double angleDeg = Math.toDegrees(angleRad);

        // 如果需要角度在[0, 360)范围内，则进行如下处理（可选）
        if (angleDeg < 0) {
            angleDeg += 360;
        }

        // 输出角度
        System.out.println("calcAngle2Point: " + angleDeg + " degrees");
        return angleDeg;
    }
    
    public static boolean isBeween(double val, double range1, double range2) {
        double min = Math.min(range1, range2);
        double max = Math.max(range1, range2);
        return val >= min && val <= max;
    }
}


