package frc.robot.Libraries;

import java.util.function.BooleanSupplier;

public class StepState {
    private AutonomousSteps m_name;
    private BooleanSupplier m_stepEnabled;

    public StepState(AutonomousSteps name, BooleanSupplier enabled) {
        m_name = name;
        m_stepEnabled = enabled;
    }

    public StepState(AutonomousSteps name) {
        this(name, () -> true);
    }

    public boolean isTrue() {
        return this.m_stepEnabled.getAsBoolean(); 
    }

    public BooleanSupplier getBooleanSupplier() {
        return this.m_stepEnabled;
    }

    public AutonomousSteps getName() {
        return this.m_name;
    }

    public String getStrName() {
        return this.m_name.toString();
    }
}