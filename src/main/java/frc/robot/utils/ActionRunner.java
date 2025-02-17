package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ActionRunner {
    class Action {
        Runnable m_init;
        Runnable m_update;
        Runnable m_onCancel;
        BooleanSupplier m_endCondition;
        BooleanSupplier m_startCondition;
        public enum STATE {
            READY,
            RUNNING,
            DONE,
        }
        STATE state = STATE.READY;
        public Action(Runnable init, Runnable update, Runnable onCancel, BooleanSupplier startCondition, BooleanSupplier endCondition) {
            m_init = init;
            m_update = update;
            m_onCancel = onCancel;
            m_endCondition = endCondition;
            m_startCondition = startCondition;
            state = STATE.READY;
        }
    
        public void update() {
            if (state == STATE.READY) {
                if  (m_init != null) {
                    m_init.run();
                }
                state = STATE.RUNNING;
            }
            else if (state == STATE.RUNNING) {
                if (m_endCondition.getAsBoolean()) {
                    state = STATE.DONE;
                }
                m_update.run();
            }
        }
        
        public boolean canStart() {
            return m_startCondition.getAsBoolean();
        }

        public boolean getIsDone() {
            return state == STATE.DONE;
        }
    
        public void cancel() {
            if (m_onCancel != null) {
                m_onCancel.run();
            }
            state = STATE.DONE;
        }

        public STATE getState() {
            return state;
        }
    }
    
    enum STATE {
        READY,
        RUNNING,
        DONE,
    }
    private STATE state = STATE.READY;
    private int m_runningActionIndex = 0;

    private List<Action> m_actions = new ArrayList<Action>();
    private List<Action> m_conditionActions = new ArrayList<Action>();

    public ActionRunner addQueueAction(Runnable init, Runnable update, Runnable onCancel, BooleanSupplier endCondition) {
        m_actions.add(new Action(init, update, onCancel, ()->true, endCondition));
        return this;
    }

    public ActionRunner addConditionAction(Runnable init, Runnable update, Runnable onCancel, BooleanSupplier startCondition, BooleanSupplier endCondition) {
        m_conditionActions.add(new Action(init, update, onCancel, startCondition, endCondition));
        return this;
    }

    public void start() {
        m_runningActionIndex = 0;
        state = STATE.RUNNING;
    }

    public void update() {
        if (state == STATE.RUNNING) {
            if (m_runningActionIndex < m_actions.size()) {
                Action action = m_actions.get(m_runningActionIndex);
                action.update();
                if (action.getIsDone()) {
                    m_runningActionIndex++;
                }
            }

            boolean isAllConditionDone = true;
            for (int i = 0; i < m_conditionActions.size(); ++i) {
                Action action = m_conditionActions.get(i);
                switch (action.getState()) {
                    case DONE:
                        break;
                    case READY:
                        isAllConditionDone = false;
                        if (action.canStart()) {
                            action.update();
                        }
                        break;
                    case RUNNING:
                        isAllConditionDone = false;
                        action.update();
                        break;
                }
            }

            SmartDashboard.putString("ActionRunner State: ", 
                String.format("isAllConditionDone %b, actions: %d/%d", isAllConditionDone, m_runningActionIndex, m_actions.size()));
            if (isAllConditionDone && m_runningActionIndex >= m_actions.size()) {
                state = STATE.DONE;
            }
        }
    }

    public boolean getIsDone() {
        return state == STATE.DONE;
    }

    public void cancel() {
        if (state == STATE.RUNNING) {
            if (m_runningActionIndex < m_actions.size()) {
                Action action = m_actions.get(m_runningActionIndex);
                action.cancel();
            }
        }
        state = STATE.DONE;
    }
}
