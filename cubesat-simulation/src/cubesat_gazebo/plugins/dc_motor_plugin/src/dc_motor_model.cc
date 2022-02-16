class DcMotor
{
    private:
        float resistance;
        float inductance;
        float velocity_constant;
        float torque_constant;
        float dc_bus_voltage;
    
    public:
        float current;
        float velocity;
        float torque;
        float duty_cycle;

    public:
        DcMotor(){}
        DcMotor(float resistance, float inductance, float velocity_constant, float torque_constant,float dc_bus_voltage)
        {
            this->resistance = resistance;
            this->inductance = inductance;
            this->velocity_constant = velocity_constant;
            this->torque_constant = torque_constant;
            this->dc_bus_voltage = dc_bus_voltage;
            this->duty_cycle = 0;
            this->current = 0;
            this->velocity = 0;
            this->torque = 0;
        }

        void propagateState(float time_step, float velocity)
        {
            this->velocity = velocity;

            current = current + time_step*(dc_bus_voltage*duty_cycle - current*resistance - velocity*velocity_constant)/inductance;
            torque = current*torque_constant;
        }
};