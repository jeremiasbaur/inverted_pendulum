class PID
{
  private:
    double* input;
    double* output;
    double* setpoint;
    double Kp, Kd, Ki, last_error, curr_sum;
    long long current_time, last_time;
    int sampling_time, array_length, counter;
    double* sum_error;
  public:
    PID(double* input, double* output, double* setpoint,
        double Kp, double Kd, double Ki, double sampling_time, int array_length);
    void setParameters(double Kp, double Kd, double Ki);
    void calculateOutput();
    void clearSumError();
};
