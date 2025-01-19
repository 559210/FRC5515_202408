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
}
