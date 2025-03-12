package frc.robot;

import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.pathplanner.lib.path.PathPlannerPath;

public final class GlobalConfig {
    public static final int version = 2025;

    private static HashMap<String, PathPlannerPath> aimPathDic = new HashMap<String,PathPlannerPath>();
    private static final int[] aimAprilTagIds = new int[] {
        6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22,
    };

    private static final String[] sourcePath = new String[]{                                         
        "source10-1_left",                                                      
        "source10-2_left",                                                      
        "source11-1_left",                                                      
        "source17-12_left",                                                     
        "source18-12_left",                                                     
        "source18-13_left",                                                     
        "source19-13_left",                                                     
        "source20-13_left",                                                     
        "source21-12_left",                                                     
        "source21-13_left",                                                     
        "source22-12_left",                                                     
        "source6-1_left",                                                       
        "source7-1_left",                                                       
        "source7-2_left",                                                       
        "source8-2_left",                                                       
        "source9-2_left",
        "source10-1_right",                                                      
        "source10-2_right",                                                      
        "source11-1_right",                                                      
        "source17-12_right",                                                     
        "source18-12_right",                                                     
        "source18-13_right",                                                     
        "source19-13_right",                                                     
        "source20-13_right",                                                     
        "source21-12_right",                                                     
        "source21-13_right",                                                     
        "source22-12_right",                                                     
        "source6-1_right",                                                       
        "source7-1_right",                                                       
        "source7-2_right",                                                       
        "source8-2_right",                                                       
        "source9-2_right",    
    };
    
    public static String getApPathName(int apId, boolean isLeft) {
        return "ap" + apId + "_" + (isLeft ? "left" : "right");
    }

    public static String getBallPathName(int apId) {
        return "ball" + apId;
    }

    public static boolean ExtractApPathName(String pathName) {
        String regex = "ap(\\d+)_(\\w+)";
        Pattern pattern = Pattern.compile(regex);
        Matcher matcher = pattern.matcher(pathName);
        if (matcher.find()) {
            String apId = matcher.group(1);
            String suffix = matcher.group(2);

            ControlPadHelper.setControlPadInfoDataInAuto(Long.parseLong(apId), suffix == "left", 3);
            return true;
        }
        else {
            return false;
        }
    }



    public static boolean init() {
        try {
            for (int i = 0; i < aimAprilTagIds.length; ++i) {
                for (int j = 0; j < 2; ++j) {
                    String pathName = getApPathName(aimAprilTagIds[i], j == 0);
                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                    System.out.println("=========== " + pathName + " loaded ===============");
                    aimPathDic.put(pathName, path);
                }

                String pathName = getBallPathName(aimAprilTagIds[i]);
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                System.out.println("=========== " + pathName + " loaded ===============");
                aimPathDic.put(pathName, path);
            }

            for (int i = 0; i < sourcePath.length; ++i) {
                String pathName = sourcePath[i];
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                System.out.println("=========== " + pathName + " loaded ===============");
                aimPathDic.put(pathName, path);
            }


            return true;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return false;
    }


    public static PathPlannerPath getAimPath(String name) {
        if (!aimPathDic.containsKey(name)) {
            return null;
        }

        return aimPathDic.get(name);
    }

    public static String[] getAllAimPathNames() {
        return (String[]) aimPathDic.keySet().toArray(new String[0]);
    }
}
