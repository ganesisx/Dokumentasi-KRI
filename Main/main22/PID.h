#ifndef PID_H 
#define PID_H 
class PID{
    public :
            PID(double p , double i , double d, int _Ts) ;

            void setTunings(double p, double i, double d);

            double createpwm( float setpoint, float feedback) ;
        
    private :
            double kp;
            double ki;
            double kd;
            float curr_e;
            float int_e;
            float dif_e;
            float prev_e;
            double u;
            float sampleTimeSec;
            int _Ts;            
};
#endif