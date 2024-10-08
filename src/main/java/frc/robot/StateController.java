package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateController {  
    private volatile static StateController instance;  
  
    private StateController() {}  
  
    public static StateController getInstance() {  
        if (instance == null) {  
            synchronized (StateController.class) {  
                if (instance == null) {  
                    instance = new StateController();  
                    instance.init();
                }  
            }  
        }  
        return instance;  
    }

    private void init() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            // SmartDashboard.putBoolean("isPresent", true);
            myAlliance = alliance.get();
            
            switch (myAlliance) {
                case Red:
                    myAllianceIndex = 0;
                    break;
                case Blue:
                    myAllianceIndex = 1;
                    break;
                default:
                    break;
            }
        }
        else {
            // SmartDashboard.putBoolean("isPresent", false);
        }

        // SmartDashboard.putString("Allience", myAlliance.toString());
    }
    public DriverStation.Alliance myAlliance = DriverStation.Alliance.Red;
    public int myAllianceIndex = 0;

    public boolean isAutoAimming = false;
    public boolean isAutoIntakeAimming = false;
    public double aimTx = 0;
    public double aimTy = 0;
    public double intakeAimTx = 0;
    public double intakeAimTy = 0;
    public boolean intakeAimStop = false;
}