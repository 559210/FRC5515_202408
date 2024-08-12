package frc.robot;


public class StateController {  
    private volatile static StateController instance;  
  
    private StateController() {}  
  
    public static StateController getInstance() {  
        if (instance == null) {  
            synchronized (StateController.class) {  
                if (instance == null) {  
                    instance = new StateController();  
                }  
            }  
        }  
        return instance;  
    }

    public boolean isAutoAimming = false;
}